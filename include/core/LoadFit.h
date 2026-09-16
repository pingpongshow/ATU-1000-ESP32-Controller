#pragma once
//
// Load estimation from magnitude-only measurements.
//
// Each tuning measurement is a known network setting plus a measured |Gamma|.
// Given enough of them at different settings, the complex load impedance
// (R + jX) is the one whose predicted |Gamma| values best agree with what was
// measured. The fit is a coarse grid over (ln R, asinh(X/50)) for both K2
// mappings, followed by a local pattern search on the best few distinct
// minima - magnitude-only data can have more than one plausible solution, so
// several are kept and the measurements pick between them.
//
// MatchPlanner then evaluates every relay combination against each estimate
// and proposes the predicted best. Both classes work in small slices (one call
// = one grid row / one inductor setting) so the firmware loop never stalls.
//

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "Network.h"
#include "RelayState.h"

namespace atu {

struct FitPoint {
  RelayState state;
  float gamma = 1.0f;
};

struct LoadEstimate {
  Cplx z{50.0f, 0.0f};
  bool inverted = false;
  float rms = 1.0f;       // RMS |Gamma| residual of the fit
};

class LoadFitter {
 public:
  static constexpr size_t kMaxPoints = 40;
  static constexpr size_t kMaxEstimates = 3;

  void begin(uint32_t freqHz, const FitPoint* pts, size_t n) {
    freqHz_ = freqHz;
    n_ = 0;
    for (size_t i = 0; i < n && n_ < kMaxPoints; ++i) {
      pts_[n_] = pts[i];
      lUh_[n_] = maskToUh(pts[i].state.lMask);
      cPf_[n_] = static_cast<float>(maskToPf(pts[i].state.cMask));
      ++n_;
    }
    row_ = 0;
    rawCount_ = 0;
    count_ = 0;
    refineIdx_ = 0;
    stage_ = n_ >= 3 ? Stage::Grid : Stage::Done;
  }

  // Returns true once the fit is complete.
  bool step() {
    switch (stage_) {
      case Stage::Grid:
        gridRow(row_++);
        if (row_ >= kRSteps) {
          selectDistinct();
          stage_ = Stage::Refine;
        }
        return false;
      case Stage::Refine:
        if (refineIdx_ < count_) {
          refine(est_[refineIdx_++]);
          return false;
        }
        finalise();
        stage_ = Stage::Done;
        return true;
      default:
        return true;
    }
  }

  size_t count() const { return count_; }
  const LoadEstimate& estimate(size_t i) const { return est_[i]; }
  const LoadEstimate* estimates() const { return est_; }

 private:
  enum class Stage : uint8_t { Grid, Refine, Done };

  // ln R over 2..5000 ohms, asinh(X/50) over +/-4000 ohms.
  static constexpr int kRSteps = 32;
  static constexpr int kXSteps = 41;
  static constexpr float kUMin = 0.6931f;    // ln 2
  static constexpr float kUMax = 8.5172f;    // ln 5000
  static constexpr float kVMax = 5.0752f;    // asinh(80)

  struct Raw { float u, v, cost; bool inv; };

  uint32_t freqHz_ = 0;
  FitPoint pts_[kMaxPoints];
  float lUh_[kMaxPoints];
  float cPf_[kMaxPoints];
  size_t n_ = 0;
  Stage stage_ = Stage::Done;
  int row_ = 0;

  static constexpr size_t kRawKeep = 16;
  Raw raw_[kRawKeep];
  size_t rawCount_ = 0;

  LoadEstimate est_[kMaxEstimates];
  float estU_[kMaxEstimates], estV_[kMaxEstimates];
  size_t count_ = 0;
  size_t refineIdx_ = 0;

  static Cplx zFrom(float u, float v) {
    return Cplx(std::exp(u), kZ0 * std::sinh(v));
  }

  float cost(float u, float v, bool inv) const {
    Cplx z = zFrom(u, v);
    const float f = static_cast<float>(freqHz_);
    float sum = 0.0f;
    for (size_t i = 0; i < n_; ++i) {
      float d = modelGammaLC(f, lUh_[i], cPf_[i], pts_[i].state.topology, inv, z) -
                pts_[i].gamma;
      sum += d * d;
    }
    return std::sqrt(sum / static_cast<float>(n_));
  }

  void offer(float u, float v, bool inv, float c) {
    if (rawCount_ < kRawKeep) {
      raw_[rawCount_++] = Raw{u, v, c, inv};
    } else {
      size_t worst = 0;
      for (size_t i = 1; i < rawCount_; ++i) {
        if (raw_[i].cost > raw_[worst].cost) worst = i;
      }
      if (c >= raw_[worst].cost) return;
      raw_[worst] = Raw{u, v, c, inv};
    }
  }

  void gridRow(int r) {
    float u = kUMin + (kUMax - kUMin) * static_cast<float>(r) / (kRSteps - 1);
    for (int m = 0; m < 2; ++m) {
      for (int x = 0; x < kXSteps; ++x) {
        float v = -kVMax + 2.0f * kVMax * static_cast<float>(x) / (kXSteps - 1);
        offer(u, v, m == 1, cost(u, v, m == 1));
      }
    }
  }

  void selectDistinct() {
    // Sort raw minima by cost, then keep ones that are not neighbours of an
    // already selected minimum.
    for (size_t i = 1; i < rawCount_; ++i) {
      Raw k = raw_[i];
      size_t j = i;
      while (j > 0 && raw_[j - 1].cost > k.cost) { raw_[j] = raw_[j - 1]; --j; }
      raw_[j] = k;
    }
    count_ = 0;
    for (size_t i = 0; i < rawCount_ && count_ < kMaxEstimates; ++i) {
      bool near = false;
      for (size_t j = 0; j < count_; ++j) {
        if (est_[j].inverted == raw_[i].inv &&
            std::fabs(estU_[j] - raw_[i].u) < 0.6f &&
            std::fabs(estV_[j] - raw_[i].v) < 0.6f) { near = true; break; }
      }
      if (near) continue;
      estU_[count_] = raw_[i].u;
      estV_[count_] = raw_[i].v;
      est_[count_].inverted = raw_[i].inv;
      est_[count_].rms = raw_[i].cost;
      ++count_;
    }
  }

  void refine(LoadEstimate& e) {
    size_t idx = static_cast<size_t>(&e - est_);
    float u = estU_[idx], v = estV_[idx];
    float best = cost(u, v, e.inverted);
    float s = 0.25f;
    for (int iter = 0; iter < 80 && s > 0.004f; ++iter) {
      static constexpr float du[4] = {1, -1, 0, 0};
      static constexpr float dv[4] = {0, 0, 1, -1};
      bool moved = false;
      for (int k = 0; k < 4; ++k) {
        float nu = u + du[k] * s, nv = v + dv[k] * s;
        if (nu < 0.0f || nu > 9.2f || nv < -6.0f || nv > 6.0f) continue;
        float c = cost(nu, nv, e.inverted);
        if (c < best) { best = c; u = nu; v = nv; moved = true; break; }
      }
      if (!moved) s *= 0.5f;
    }
    estU_[idx] = u;
    estV_[idx] = v;
    e.z = zFrom(u, v);
    e.rms = best;
  }

  void finalise() {
    // Drop minima that converged onto each other, then order by residual.
    size_t w = 0;
    for (size_t i = 0; i < count_; ++i) {
      bool dup = false;
      for (size_t j = 0; j < w; ++j) {
        if (est_[j].inverted == est_[i].inverted &&
            std::fabs(estU_[j] - estU_[i]) < 0.1f &&
            std::fabs(estV_[j] - estV_[i]) < 0.1f) { dup = true; break; }
      }
      if (dup) continue;
      est_[w] = est_[i];
      estU_[w] = estU_[i];
      estV_[w] = estV_[i];
      ++w;
    }
    count_ = w;
    for (size_t i = 1; i < count_; ++i) {
      LoadEstimate k = est_[i];
      size_t j = i;
      while (j > 0 && est_[j - 1].rms > k.rms) { est_[j] = est_[j - 1]; --j; }
      est_[j] = k;
    }
  }
};

// -----------------------------------------------------------------------------

struct PlannedState {
  RelayState state;
  float predicted = 1.0f;
};

class MatchPlanner {
 public:
  static constexpr size_t kMaxPlans = 4;

  // Only estimates whose residual is below maxRms are planned for.
  void begin(uint32_t freqHz, const LoadEstimate* est, size_t nEst, float maxRms) {
    freqHz_ = freqHz;
    nEst_ = 0;
    for (size_t i = 0; i < nEst && nEst_ < LoadFitter::kMaxEstimates; ++i) {
      if (est[i].rms <= maxRms) est_[nEst_++] = est[i];
    }
    for (size_t i = 0; i < LoadFitter::kMaxEstimates; ++i) topCount_[i] = 0;
    lMask_ = 0;
    count_ = 0;
  }

  // Returns true once every combination has been evaluated.
  bool step() {
    if (nEst_ == 0 || lMask_ >= kComboCount) {
      if (count_ == 0) collect();
      return true;
    }
    RelayState s;
    s.lMask = static_cast<uint8_t>(lMask_);
    const float f = static_cast<float>(freqHz_);
    const float lUh = maskToUh(s.lMask);
    for (int c = 0; c < kComboCount; ++c) {
      s.cMask = static_cast<uint8_t>(c);
      const float cPf = cLadder().valueAt[cLadder().indexOf[c]];
      for (int t = 0; t < 2; ++t) {
        s.topology = (t == 1);
        for (size_t e = 0; e < nEst_; ++e) {
          offer(e, s, modelGammaLC(f, lUh, cPf, s.topology, est_[e].inverted, est_[e].z));
        }
      }
    }
    ++lMask_;
    if (lMask_ >= kComboCount) {
      collect();
      return true;
    }
    return false;
  }

  size_t count() const { return count_; }
  const PlannedState& plan(size_t i) const { return plans_[i]; }

 private:
  static constexpr size_t kTopKeep = 8;

  uint32_t freqHz_ = 0;
  LoadEstimate est_[LoadFitter::kMaxEstimates];
  size_t nEst_ = 0;
  int lMask_ = 0;

  PlannedState top_[LoadFitter::kMaxEstimates][kTopKeep];
  size_t topCount_[LoadFitter::kMaxEstimates];

  PlannedState plans_[kMaxPlans];
  size_t count_ = 0;

  void offer(size_t e, const RelayState& s, float g) {
    PlannedState* t = top_[e];
    size_t& n = topCount_[e];
    if (n == kTopKeep && g >= t[n - 1].predicted) return;
    size_t pos = (n < kTopKeep) ? n++ : kTopKeep - 1;
    while (pos > 0 && t[pos - 1].predicted > g) { t[pos] = t[pos - 1]; --pos; }
    t[pos].state = s;
    t[pos].predicted = g;
  }

  static bool farApart(const RelayState& a, const RelayState& b) {
    if (a.topology != b.topology) return true;
    int dl = static_cast<int>(lLadder().indexOf[a.lMask]) - lLadder().indexOf[b.lMask];
    int dc = static_cast<int>(cLadder().indexOf[a.cMask]) - cLadder().indexOf[b.cMask];
    return std::abs(dl) > 4 || std::abs(dc) > 4;
  }

  void push(const PlannedState& p) {
    if (count_ >= kMaxPlans) return;
    for (size_t i = 0; i < count_; ++i) {
      if (plans_[i].state == p.state) return;
    }
    plans_[count_++] = p;
  }

  void collect() {
    count_ = 0;
    // Best prediction for each estimate, most plausible estimate first.
    for (size_t e = 0; e < nEst_; ++e) {
      if (topCount_[e] > 0) push(top_[e][0]);
    }
    // Plus a distinct runner-up for the most plausible estimate, which covers
    // the model being slightly off in a way that moves the optimum.
    if (nEst_ > 0) {
      for (size_t i = 1; i < topCount_[0]; ++i) {
        if (farApart(top_[0][i].state, top_[0][0].state)) {
          push(top_[0][i]);
          break;
        }
      }
    }
  }
};

}  // namespace atu
