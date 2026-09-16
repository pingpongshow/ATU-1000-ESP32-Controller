// Desktop test harness for the tuning maths.
//
//   pio test -e native
//
// The search is driven against a simulated tuner whose circuit deliberately
// does NOT match the firmware's own model: component tolerances, wiring
// inductance, stray capacitance, finite inductor Q and measurement noise are
// all switched on. Each load's best achievable |Gamma| is found by brute force
// over all 32,768 settings, so results are scored against what the hardware
// could actually reach rather than an arbitrary threshold.
//
// The previous bit-flip search is reimplemented here too, so the benchmark
// prints a like-for-like comparison.

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "LoadFit.h"
#include "Network.h"
#include "Solver.h"
#include "TuneSearch.h"

using namespace atu;

namespace {

struct Plant {
  uint32_t freqHz;
  Cplx load;
  bool inverted;
  Parasitics par;
  float noiseSigma;   // per quick measurement
  std::mt19937* rng;

  float trueGamma(const RelayState& s) const {
    return modelGamma(freqHz, s, inverted, load, par);
  }

  float measure(const RelayState& s, bool precise, float& noiseOut) const {
    // Precise readings average three times as many samples.
    float sigma = precise ? noiseSigma / std::sqrt(3.0f) : noiseSigma;
    std::normal_distribution<float> n(0.0f, sigma);
    float g = trueGamma(s) + n(*rng);
    noiseOut = sigma;
    if (g < 0.0f) g = std::fabs(g);
    return g > 1.0f ? 1.0f : g;
  }

  float bestAchievable() const {
    float best = 1.0f;
    RelayState s;
    for (int l = 0; l < 128; ++l) {
      s.lMask = static_cast<uint8_t>(l);
      for (int c = 0; c < 128; ++c) {
        s.cMask = static_cast<uint8_t>(c);
        for (int t = 0; t < 2; ++t) {
          s.topology = (t == 1);
          float g = trueGamma(s);
          if (g < best) best = g;
        }
      }
    }
    return best;
  }
};

struct RunResult {
  float achieved;   // true |Gamma| of the final state
  int steps;
};

RunResult runSearch(const Plant& plant, bool useModel, bool freqKnown) {
  SearchParams p;
  p.freqHz = freqKnown ? plant.freqHz : 0;
  p.useModel = useModel;
  p.maxSteps = 160;
  p.refinePasses = 3;

  TuneSearch search;
  RelayState entry;
  search.start(p, entry, nullptr);

  int guard = 0;
  for (;;) {
    Probe probe;
    SearchStatus st = search.next(probe);
    if (st == SearchStatus::Done) break;
    if (st == SearchStatus::Busy) {
      if (++guard > 100000) break;
      continue;
    }
    float noise = 0.0f;
    float g = plant.measure(probe.state, probe.precise, noise);
    search.report(true, g, noise);
  }
  return RunResult{plant.trueGamma(search.best()), search.steps()};
}

// The v2.0 firmware search: seeds, MSB-first bit flips on C then L, then
// single-bit refinement. Kept here purely as a benchmark reference.
RunResult runLegacy(const Plant& plant) {
  int steps = 0;
  float dummy;
  auto meas = [&](const RelayState& s) {
    ++steps;
    return plant.measure(s, false, dummy);
  };

  std::vector<RelayState> cands;
  auto push = [&](const RelayState& s) {
    for (auto& c : cands) if (c == s) return;
    cands.push_back(s);
  };
  RelayState entry;
  push(entry);
  if (const BandInfo* b = findBand(plant.freqHz)) {
    RelayState s;
    s.lMask = b->typicalLMask;
    s.cMask = b->typicalCMask;
    s.topology = b->typicalTopology;
    push(s);
  }
  RelayState seeds[kSeedResistanceCount];
  for (int t = 0; t < 2; ++t) {
    size_t n = buildAnalyticSeeds(plant.freqHz, t == 1, seeds, kSeedResistanceCount);
    for (size_t i = 0; i < n; ++i) push(seeds[i]);
  }

  RelayState best = entry;
  float bestG = 1.0f;
  for (auto& c : cands) {
    float g = meas(c);
    if (g < bestG) { bestG = g; best = c; }
  }
  const float target = 0.0909f;
  for (int onC = 1; onC >= 0 && bestG > target; --onC) {
    for (int bit = 6; bit >= 0 && bestG > target; --bit) {
      RelayState t = best;
      if (onC) t.cMask ^= 1 << bit; else t.lMask ^= 1 << bit;
      float g = meas(t);
      if (g + 0.003f < bestG) { bestG = g; best = t; }
    }
  }
  for (int pass = 0; pass < 3 && bestG > target; ++pass) {
    bool improved = false;
    for (int onC = 1; onC >= 0; --onC) {
      for (int bit = 0; bit < 7 && bestG > target; ++bit) {
        RelayState t = best;
        if (onC) t.cMask ^= 1 << bit; else t.lMask ^= 1 << bit;
        float g = meas(t);
        if (g + 0.002f < bestG) { bestG = g; best = t; improved = true; }
      }
    }
    if (!improved) break;
  }
  meas(best);
  return RunResult{plant.trueGamma(best), steps};
}

const uint32_t kTestFreqs[] = {1850000, 3650000, 7100000, 10120000, 14200000,
                               18100000, 21200000, 24940000, 28500000};

Plant randomPlant(std::mt19937& rng) {
  std::uniform_int_distribution<int> fi(0, sizeof(kTestFreqs) / sizeof(kTestFreqs[0]) - 1);
  std::uniform_real_distribution<float> u01(0.0f, 1.0f);
  Plant p;
  p.freqHz = kTestFreqs[fi(rng)];
  float r = std::exp(std::log(5.0f) + u01(rng) * (std::log(1500.0f) - std::log(5.0f)));
  float x = 50.0f * std::sinh((u01(rng) * 2.0f - 1.0f) * 3.2f);
  p.load = Cplx(r, x);
  p.inverted = u01(rng) < 0.5f;
  p.par.lScale = 0.93f + 0.14f * u01(rng);
  p.par.cScale = 0.95f + 0.10f * u01(rng);
  p.par.seriesLUh = 0.02f + 0.04f * u01(rng);
  p.par.strayInPf = 5.0f + 10.0f * u01(rng);
  p.par.strayOutPf = 5.0f + 10.0f * u01(rng);
  p.par.inductorQ = 120.0f + 120.0f * u01(rng);
  p.noiseSigma = 0.008f;
  p.rng = &rng;
  return p;
}

// -----------------------------------------------------------------------------

void test_ladder_is_value_ordered() {
  const Ladder& l = lLadder();
  const Ladder& c = cLadder();
  for (int i = 1; i < kComboCount; ++i) {
    TEST_ASSERT_TRUE(l.valueAt[i] >= l.valueAt[i - 1]);
    TEST_ASSERT_TRUE(c.valueAt[i] >= c.valueAt[i - 1]);
    TEST_ASSERT_EQUAL_UINT8(i, l.indexOf[l.maskAt[i]]);
    TEST_ASSERT_EQUAL_UINT8(i, c.indexOf[c.maskAt[i]]);
  }
}

void test_matched_load_straight_through() {
  RelayState zero;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, modelGamma(14000000, zero, false, Cplx(50, 0)));
}

void test_analytic_design_matches_model() {
  // The closed-form L-match must produce a match in the circuit model for the
  // topology that puts the capacitor across the higher-resistance side.
  for (float r : {12.5f, 200.0f, 800.0f}) {
    float lUh, cPf;
    TEST_ASSERT_TRUE(designLNetwork(7000000, r, lUh, cPf));
    bool capsAtLoad = r > kZ0;
    float g = gammaOf(networkInputZ(7000000.0f, lUh, cPf, capsAtLoad, Cplx(r, 0), Parasitics()));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, g);
  }
}

void test_fit_recovers_load() {
  const uint32_t f = 14200000;
  const Cplx load(180.0f, -95.0f);
  RelayState seeds[16];
  std::vector<FitPoint> pts;
  for (int t = 0; t < 2; ++t) {
    size_t n = buildAnalyticSeeds(f, t == 1, seeds, kSeedResistanceCount);
    for (size_t i = 0; i < n; ++i) {
      FitPoint fp;
      fp.state = seeds[i];
      fp.gamma = modelGamma(f, seeds[i], false, load);
      pts.push_back(fp);
    }
  }
  LoadFitter fitter;
  fitter.begin(f, pts.data(), pts.size());
  while (!fitter.step()) {}
  TEST_ASSERT_TRUE(fitter.count() > 0);

  // Magnitude-only data can admit more than one load; the true one must be
  // among the estimates and the planner must reach a match for it.
  bool found = false;
  for (size_t i = 0; i < fitter.count(); ++i) {
    const LoadEstimate& e = fitter.estimate(i);
    if (!e.inverted && std::fabs(e.z.real() - load.real()) < 15.0f &&
        std::fabs(e.z.imag() - load.imag()) < 15.0f) found = true;
  }
  TEST_ASSERT_TRUE_MESSAGE(found, "true load not among the fit estimates");

  MatchPlanner planner;
  planner.begin(f, fitter.estimates(), fitter.count(), 0.12f);
  while (!planner.step()) {}
  float best = 1.0f;
  for (size_t i = 0; i < planner.count(); ++i) {
    float g = modelGamma(f, planner.plan(i).state, false, load);
    if (g < best) best = g;
  }
  TEST_ASSERT_TRUE(best < 0.1f);
}

void test_search_never_worse_than_start() {
  std::mt19937 rng(7);
  for (int i = 0; i < 40; ++i) {
    Plant p = randomPlant(rng);
    RelayState zero;
    float start = p.trueGamma(zero);
    RunResult r = runSearch(p, true, true);
    TEST_ASSERT_TRUE(r.achieved <= start + 0.03f);
    TEST_ASSERT_TRUE(r.steps <= 162);
  }
}

struct Stats {
  int runs = 0, success = 0, good = 0, nearOptimal = 0;
  long steps = 0;
  void add(const RunResult& r, float optimum) {
    ++runs;
    steps += r.steps;
    if (r.achieved <= 0.0909f) ++success;       // SWR 1.2
    if (r.achieved <= 0.2f) ++good;             // SWR 1.5
    // Stopping at the target is correct behaviour, so "near optimal" means
    // within 0.03 of the best achievable or at the target, whichever is worse.
    if (r.achieved <= std::max(optimum + 0.03f, 0.0909f)) ++nearOptimal;
  }
  void print(const char* name) const {
    printf("  %-26s runs %3d  SWR<=1.2 %5.1f%%  SWR<=1.5 %5.1f%%  near-optimal %5.1f%%  mean steps %5.1f\n",
           name, runs, 100.0 * success / runs, 100.0 * good / runs,
           100.0 * nearOptimal / runs, static_cast<double>(steps) / runs);
  }
  float goodRate() const { return static_cast<float>(good) / runs; }
  float nearRate() const { return static_cast<float>(nearOptimal) / runs; }
  float meanSteps() const { return static_cast<float>(steps) / runs; }
};

void test_benchmark_against_simulated_tuner() {
  std::mt19937 rng(12345);
  Stats model, noModel, noFreq, legacy;
  int matchable = 0, skipped = 0;

  for (int i = 0; i < 300; ++i) {
    Plant p = randomPlant(rng);
    float optimum = p.bestAchievable();
    if (optimum > 0.2f) { ++skipped; continue; }   // outside the tuner's range
    ++matchable;
    model.add(runSearch(p, true, true), optimum);
    noModel.add(runSearch(p, false, true), optimum);
    noFreq.add(runSearch(p, true, false), optimum);
    legacy.add(runLegacy(p), optimum);
  }

  printf("\nBenchmark: %d matchable loads (%d outside the tuner's range skipped)\n",
         matchable, skipped);
  model.print("model fit + pattern");
  noModel.print("pattern only");
  noFreq.print("no frequency (grid)");
  legacy.print("v2.0 bit-flip search");

  TEST_ASSERT_TRUE(model.goodRate() >= 0.97f);
  TEST_ASSERT_TRUE(model.nearRate() >= 0.95f);
  TEST_ASSERT_TRUE(model.meanSteps() <= 35.0f);
  TEST_ASSERT_TRUE(noFreq.goodRate() >= 0.93f);
  TEST_ASSERT_TRUE(noFreq.nearRate() >= 0.90f);
  TEST_ASSERT_TRUE(noModel.goodRate() >= legacy.goodRate());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_ladder_is_value_ordered);
  RUN_TEST(test_matched_load_straight_through);
  RUN_TEST(test_analytic_design_matches_model);
  RUN_TEST(test_fit_recovers_load);
  RUN_TEST(test_search_never_worse_than_start);
  RUN_TEST(test_benchmark_against_simulated_tuner);
  return UNITY_END();
}
