#pragma once
//
// Tuning search strategy, independent of hardware.
//
// The caller drives it as a loop:
//
//   search.start(params, entryState, hint);
//   for (;;) {
//     Probe p;
//     SearchStatus st = search.next(p);
//     if (st == SearchStatus::Done) break;
//     if (st == SearchStatus::Busy) continue;     // computing, call again
//     ...apply p.state, wait for settle, measure...
//     search.report(valid, gamma, gammaNoise);
//   }
//
// Phases:
//   Baseline  measure the state we started from
//   Seeds     memory hint, band typical, analytic L-match seeds (or a coarse
//             grid when the frequency is unknown)
//   Fit/Plan  estimate the complex load (and the frequency, if unknown) from
//             the seed measurements and compute the relay combination the
//             model predicts is best (LoadFit.h)
//   Model     measure the planned states; one refit if they disappoint
//   Pattern   noise-aware Hooke-Jeeves pattern search in value order
//             (+/-step on L and C including diagonals), step halving 16 -> 1
//   Confirm   precise re-measurement of the winner
//
// Scores are |Gamma| rather than SWR: it is bounded, roughly linear in the
// detector voltages, and does not flatten out far from a match the way SWR
// does once it saturates.
//
// Free of Arduino headers; the desktop test harness in test/ drives it against
// a simulated tuner.
//

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "Components.h"
#include "LoadFit.h"
#include "Network.h"
#include "RelayState.h"
#include "Solver.h"

namespace atu {

struct SearchParams {
  uint32_t freqHz = 0;          // 0 = unknown
  float targetGamma = 0.0909f;  // SWR 1.2
  float goodGamma = 0.2f;       // SWR 1.5
  uint8_t refinePasses = 3;     // re-expansions after converging at step 1
  uint16_t maxSteps = 160;
  bool useModel = true;
};

struct Probe {
  RelayState state;
  bool precise = false;   // caller should average more samples
};

enum class SearchStatus : uint8_t { Probe, Busy, Done };

class TuneSearch {
 public:
  void start(const SearchParams& params, const RelayState& entry, const RelayState* hint) {
    p_ = params;
    entry_ = normalise(entry);
    samples_.clear();
    samples_.reserve(p_.maxSteps + 4);
    seeds_.clear();
    modelQueue_.clear();
    neighbours_.clear();

    best_ = entry_;
    bestGamma_ = 1.0f;
    bestNoise_ = 0.0f;
    bestPrecise_ = false;
    baselineGamma_ = 1.0f;
    baselineValid_ = false;
    finalGamma_ = 1.0f;
    finalValid_ = false;
    confirmTries_ = 0;
    steps_ = 0;
    seedIdx_ = 0;
    modelRound_ = 0;
    modelUsed_ = false;
    haveFit_ = false;
    modelFreq_ = 0;
    lastDir_ = currentDir_ = verifyDir_ = -2;
    haveVerify_ = false;
    rebasePending_ = false;
    awaiting_ = Await::None;

    buildSeeds(hint);
    planned_ = static_cast<uint16_t>(1 + seeds_.size() + (p_.useModel ? 4 : 0) + 24 + 1);
    if (planned_ > p_.maxSteps) planned_ = p_.maxSteps;
    phase_ = Phase::Baseline;
  }

  SearchStatus next(Probe& out) {
    if (awaiting_ != Await::None) {   // called twice without a report
      out = lastProbe_;
      return SearchStatus::Probe;
    }
    for (int guard = 0; guard < 512; ++guard) {
      if (phase_ == Phase::Done) return SearchStatus::Done;
      if (steps_ >= p_.maxSteps && phase_ != Phase::Confirm) enterConfirm();

      switch (phase_) {
        case Phase::Baseline:
          return emit(out, entry_, false, Await::Baseline);

        case Phase::Seeds: {
          if (seedIdx_ >= seeds_.size()) { afterSeeds(); continue; }
          RelayState s = seeds_[seedIdx_++];
          if (find(s)) continue;
          return emit(out, s, false, Await::Seed);
        }

        case Phase::Fit:
          if (!fitter_.step()) return SearchStatus::Busy;
          afterFit();
          continue;

        case Phase::Plan:
          if (!planner_.step()) return SearchStatus::Busy;
          afterPlan();
          continue;

        case Phase::Model: {
          if (modelQueue_.empty()) { afterModel(); continue; }
          RelayState s = modelQueue_.front();
          modelQueue_.erase(modelQueue_.begin());
          if (find(s)) continue;
          return emit(out, s, false, Await::Model);
        }

        case Phase::Pattern:
          if (patternNext(out)) return SearchStatus::Probe;
          continue;

        case Phase::Confirm:
          return emit(out, best_, true, Await::Confirm);

        default:
          return SearchStatus::Done;
      }
    }
    return SearchStatus::Busy;
  }

  void report(bool valid, float gamma, float noise) {
    Await a = awaiting_;
    awaiting_ = Await::None;
    ++steps_;

    float g = valid ? clampGamma(gamma) : 1.0f;
    float n = (valid && noise > 0.0f) ? noise : 0.0f;
    const RelayState& s = lastProbe_.state;
    bool precise = lastProbe_.precise;
    record(s, g, n, valid, precise);

    switch (a) {
      case Await::Baseline:
        baselineGamma_ = g;
        baselineValid_ = valid;
        setBest(s, g, n, false);
        phase_ = (valid && g <= p_.targetGamma) ? Phase::Confirm : Phase::Seeds;
        break;

      case Await::Seed:
      case Await::Model:
        if (g < bestGamma_) setBest(s, g, n, false);
        if (bestGamma_ <= p_.targetGamma) enterConfirm();
        break;

      case Await::Pattern:
        consider(s, g, n, precise);
        break;

      case Await::Verify:
        consider(s, g, n, true);
        break;

      case Await::Rebase:
        bestGamma_ = g;
        bestNoise_ = n;
        bestPrecise_ = true;
        break;

      case Await::Confirm: {
        bool flaky = valid && g > bestGamma_ + std::fmax(0.05f, 3.0f * (n + bestNoise_));
        if (flaky && confirmTries_ == 0) {
          ++confirmTries_;       // one more look before believing it
          break;
        }
        finalGamma_ = g;
        finalValid_ = valid;
        phase_ = Phase::Done;
        break;
      }

      default:
        break;
    }
  }

  bool done() const { return phase_ == Phase::Done; }
  const RelayState& best() const { return best_; }
  float bestGamma() const { return bestGamma_; }
  // The confirm measurement if there was one, else the best seen.
  float finalGamma() const { return finalValid_ ? finalGamma_ : bestGamma_; }
  bool anyValid() const {
    for (size_t i = 0; i < samples_.size(); ++i) {
      if (samples_[i].valid) return true;
    }
    return false;
  }
  float baselineGamma() const { return baselineGamma_; }
  bool baselineValid() const { return baselineValid_; }
  bool modelUsed() const { return modelUsed_; }
  // Frequency the load model was fitted at (estimated when none was given).
  uint32_t modelFrequency() const { return modelUsed_ ? modelFreq_ : 0; }
  uint16_t steps() const { return steps_; }
  uint16_t plannedSteps() const { return planned_ > steps_ ? planned_ : steps_ + 1; }

  const char* phaseName() const {
    switch (phase_) {
      case Phase::Baseline: return "baseline";
      case Phase::Seeds: return "seeds";
      case Phase::Fit: return "fit";
      case Phase::Plan: return "plan";
      case Phase::Model: return "model";
      case Phase::Pattern: return "search";
      case Phase::Confirm: return "confirm";
      default: return "done";
    }
  }

 private:
  enum class Phase : uint8_t { Baseline, Seeds, Fit, Plan, Model, Pattern, Confirm, Done };
  enum class Await : uint8_t { None, Baseline, Seed, Model, Pattern, Verify, Rebase, Confirm };

  struct Sample {
    RelayState state;
    float gamma;
    float noise;
    bool valid;
    bool precise;
  };

  SearchParams p_;
  Phase phase_ = Phase::Done;
  Await awaiting_ = Await::None;
  Probe lastProbe_;

  RelayState entry_;
  RelayState best_;
  float bestGamma_ = 1.0f;
  float bestNoise_ = 0.0f;
  bool bestPrecise_ = false;

  float baselineGamma_ = 1.0f;
  bool baselineValid_ = false;
  float finalGamma_ = 1.0f;
  bool finalValid_ = false;
  uint8_t confirmTries_ = 0;

  uint16_t steps_ = 0;
  uint16_t planned_ = 1;

  std::vector<Sample> samples_;
  std::vector<RelayState> seeds_;
  size_t seedIdx_ = 0;

  LoadFitter fitter_;
  LoadFitter bestFit_;
  bool haveFit_ = false;
  std::vector<FitPoint> fitPts_;
  size_t fitFreqIdx_ = 0;
  uint32_t fitFreq_ = 0;
  uint32_t modelFreq_ = 0;
  MatchPlanner planner_;
  std::vector<RelayState> modelQueue_;
  uint8_t modelRound_ = 0;
  bool modelUsed_ = false;
  float predictedBest_ = 1.0f;

  int patStep_ = 1;
  uint8_t restartsLeft_ = 0;
  int8_t lastDir_ = -2;
  int8_t currentDir_ = -2;
  int8_t verifyDir_ = -2;
  size_t nbIdx_ = 0;
  bool haveVerify_ = false;
  RelayState verifyState_;
  bool rebasePending_ = false;

  // -- helpers ----------------------------------------------------------------

  static float clampGamma(float g) {
    if (!(g >= 0.0f)) return 1.0f;
    return g > 1.0f ? 1.0f : g;
  }

  static RelayState normalise(const RelayState& s) {
    RelayState t = s;
    t.bypass = false;
    t.lMask &= 0x7F;
    t.cMask &= 0x7F;
    return t;
  }

  SearchStatus emit(Probe& out, const RelayState& s, bool precise, Await a) {
    out.state = s;
    out.precise = precise;
    lastProbe_ = out;
    awaiting_ = a;
    return SearchStatus::Probe;
  }

  const Sample* find(const RelayState& s) const {
    for (size_t i = 0; i < samples_.size(); ++i) {
      if (samples_[i].state == s) return &samples_[i];
    }
    return nullptr;
  }

  void record(const RelayState& s, float g, float n, bool valid, bool precise) {
    for (size_t i = 0; i < samples_.size(); ++i) {
      if (samples_[i].state == s) {
        // A precise reading always wins; a quick one never overwrites it.
        if (precise || !samples_[i].precise) samples_[i] = Sample{s, g, n, valid, precise};
        return;
      }
    }
    samples_.push_back(Sample{s, g, n, valid, precise});
  }

  void setBest(const RelayState& s, float g, float n, bool precise) {
    best_ = s;
    bestGamma_ = g;
    bestNoise_ = n;
    bestPrecise_ = precise;
  }

  void enterConfirm() {
    phase_ = Phase::Confirm;
    haveVerify_ = false;
  }

  size_t validCount() const {
    size_t n = 0;
    for (size_t i = 0; i < samples_.size(); ++i) {
      if (samples_[i].valid) ++n;
    }
    return n;
  }

  // -- seeds ------------------------------------------------------------------

  void pushSeed(const RelayState& s) {
    RelayState t = normalise(s);
    if (t == entry_) return;
    for (size_t i = 0; i < seeds_.size(); ++i) {
      if (seeds_[i] == t) return;
    }
    seeds_.push_back(t);
  }

  void buildSeeds(const RelayState* hint) {
    if (hint && !hint->bypass) pushSeed(*hint);

    if (p_.freqHz >= 100000UL) {
      if (const BandInfo* band = findBand(p_.freqHz)) {
        RelayState s;
        s.lMask = band->typicalLMask;
        s.cMask = band->typicalCMask;
        s.topology = band->typicalTopology;
        pushSeed(s);
      }
      RelayState seeds[kSeedResistanceCount];
      for (uint8_t t = 0; t < 2; ++t) {
        size_t n = buildAnalyticSeeds(p_.freqHz, t == 1, seeds, kSeedResistanceCount);
        for (size_t i = 0; i < n; ++i) pushSeed(seeds[i]);
      }
    } else {
      // No frequency: a coarse grid in value order across both topologies.
      static constexpr uint8_t kGrid[] = {18, 50, 96};
      for (uint8_t t = 0; t < 2; ++t) {
        for (uint8_t li : kGrid) {
          for (uint8_t ci : kGrid) {
            RelayState s;
            s.lMask = lLadder().maskAt[li];
            s.cMask = cLadder().maskAt[ci];
            s.topology = (t == 1);
            pushSeed(s);
          }
        }
      }
    }

    RelayState zero;
    zero.topology = entry_.topology;
    pushSeed(zero);
  }

  // -- model ------------------------------------------------------------------

  // With no frequency from CAT, the fit also estimates it: the seed grid is
  // fitted at every band centre and the frequency whose model explains the
  // measurements best is used for planning.
  void startFit() {
    fitPts_.clear();
    for (size_t i = 0; i < samples_.size() && fitPts_.size() < LoadFitter::kMaxPoints; ++i) {
      if (!samples_[i].valid) continue;
      FitPoint fp;
      fp.state = samples_[i].state;
      fp.gamma = samples_[i].gamma;
      fitPts_.push_back(fp);
    }
    fitFreqIdx_ = 0;
    haveFit_ = false;
    beginFitAt(p_.freqHz >= 100000UL ? p_.freqHz : kBands[0].lowHz / 2 + kBands[0].highHz / 2);
    phase_ = Phase::Fit;
  }

  void beginFitAt(uint32_t freqHz) {
    fitFreq_ = freqHz;
    fitter_.begin(freqHz, fitPts_.data(), fitPts_.size());
  }

  bool modelPossible() const {
    return p_.useModel && validCount() >= (p_.freqHz >= 100000UL ? 5u : 8u);
  }

  void afterSeeds() {
    if (modelPossible()) {
      startFit();
      return;
    }
    enterPattern(p_.freqHz >= 100000UL ? 8 : 16);
  }

  void afterFit() {
    if (fitter_.count() > 0 && (!haveFit_ || fitter_.estimate(0).rms < bestFit_.estimate(0).rms)) {
      bestFit_ = fitter_;
      modelFreq_ = fitFreq_;
      haveFit_ = true;
    }
    if (p_.freqHz < 100000UL && ++fitFreqIdx_ < kBandCount) {
      beginFitAt(kBands[fitFreqIdx_].lowHz / 2 + kBands[fitFreqIdx_].highHz / 2);
      return;   // stay in Phase::Fit
    }

    // A residual this large means the model does not describe what was
    // measured (bad calibration, odd load); do not trust its predictions.
    const float kMaxRms = 0.12f;
    if (!haveFit_ || bestFit_.estimate(0).rms > kMaxRms) {
      enterPattern(8);
      return;
    }
    planner_.begin(modelFreq_, bestFit_.estimates(), bestFit_.count(), kMaxRms);
    phase_ = Phase::Plan;
  }

  void afterPlan() {
    modelQueue_.clear();
    predictedBest_ = 1.0f;
    for (size_t i = 0; i < planner_.count(); ++i) {
      modelQueue_.push_back(planner_.plan(i).state);
      if (planner_.plan(i).predicted < predictedBest_) predictedBest_ = planner_.plan(i).predicted;
    }
    modelUsed_ = modelUsed_ || !modelQueue_.empty();
    phase_ = Phase::Model;
  }

  void afterModel() {
    // If the plan fell short of its prediction, refit once with the new points
    // included - they sit right where the model is least certain.
    if (modelRound_ == 0 && bestGamma_ > p_.targetGamma &&
        bestGamma_ > predictedBest_ + 0.03f && modelPossible()) {
      ++modelRound_;
      startFit();
      return;
    }
    enterPattern(bestGamma_ < 0.33f ? 2 : 8);
  }

  // -- pattern search -----------------------------------------------------------
  //
  // Hooke-Jeeves style: moves in value order on L and C (axes, then
  // diagonals, which follow the L/C valley), a successful direction is tried
  // again first, and the step halves when a whole neighbourhood fails. Once it
  // converges at step 1 it re-expands to step 4 up to `refinePasses` times to
  // climb out of small local minima caused by quantisation and noise.

  static constexpr int8_t kMoves[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

  struct Neighbour {
    RelayState state;
    int8_t dir;         // index into kMoves, -1 = topology flip
  };
  std::vector<Neighbour> neighbours_;

  void enterPattern(int step) {
    patStep_ = step;
    restartsLeft_ = p_.refinePasses;
    lastDir_ = -2;
    phase_ = Phase::Pattern;
    buildNeighbours();
  }

  void addNeighbour(const RelayState& t, int8_t dir) {
    if (t == best_) return;
    for (size_t i = 0; i < neighbours_.size(); ++i) {
      if (neighbours_[i].state == t) return;
    }
    neighbours_.push_back(Neighbour{t, dir});
  }

  void buildNeighbours() {
    neighbours_.clear();
    nbIdx_ = 0;
    haveVerify_ = false;
    if (patStep_ <= 2 && !bestPrecise_) rebasePending_ = true;

    const int li = lLadder().indexOf[best_.lMask];
    const int ci = cLadder().indexOf[best_.cMask];
    auto moved = [&](int dir) {
      RelayState t = best_;
      t.lMask = lLadder().maskAt[clampIndex(li + kMoves[dir][0] * patStep_)];
      t.cMask = cLadder().maskAt[clampIndex(ci + kMoves[dir][1] * patStep_)];
      return t;
    };

    if (lastDir_ >= 0) addNeighbour(moved(lastDir_), lastDir_);
    for (int d = 0; d < 8; ++d) addNeighbour(moved(d), static_cast<int8_t>(d));
    if (patStep_ >= 4) {
      RelayState t = best_;
      t.topology = !t.topology;
      addNeighbour(t, -1);
    }
  }

  static int clampIndex(int i) {
    if (i < 0) return 0;
    if (i > kComboCount - 1) return kComboCount - 1;
    return i;
  }

  // Emits the next pattern probe, or advances state without measuring and
  // returns false.
  bool patternNext(Probe& out) {
    if (rebasePending_) {
      rebasePending_ = false;
      emit(out, best_, true, Await::Rebase);
      return true;
    }
    if (haveVerify_) {
      haveVerify_ = false;
      emit(out, verifyState_, true, Await::Verify);
      return true;
    }
    if (nbIdx_ >= neighbours_.size()) {
      lastDir_ = -2;
      if (patStep_ > 1) {
        patStep_ /= 2;
        buildNeighbours();
      } else if (restartsLeft_ > 0 && bestGamma_ > p_.targetGamma) {
        --restartsLeft_;
        patStep_ = 4;
        buildNeighbours();
      } else {
        enterConfirm();
      }
      return false;
    }

    const Neighbour& nb = neighbours_[nbIdx_++];
    currentDir_ = nb.dir;
    bool precise = patStep_ <= 2;
    const Sample* c = find(nb.state);
    if (c && (c->precise || !precise)) {
      consider(nb.state, c->gamma, c->noise, c->precise);
      return false;
    }
    emit(out, nb.state, precise, Await::Pattern);
    return true;
  }

  bool improves(float g, float n) const {
    float margin = 1.5f * std::sqrt(n * n + bestNoise_ * bestNoise_);
    if (margin < 0.003f) margin = 0.003f;
    return g + margin < bestGamma_;
  }

  void consider(const RelayState& s, float g, float n, bool precise) {
    if (!improves(g, n)) return;
    if (!precise) {
      // Winner's curse: one lucky quick reading must not become the new best.
      verifyState_ = s;
      verifyDir_ = currentDir_;
      haveVerify_ = true;
      return;
    }
    if (s == verifyState_) currentDir_ = verifyDir_;
    setBest(s, g, n, true);
    if (bestGamma_ <= p_.targetGamma) {
      enterConfirm();
      return;
    }
    lastDir_ = currentDir_;
    buildNeighbours();
  }
};

}  // namespace atu
