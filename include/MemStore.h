#pragma once
//
// Frequency memory.
//
// Fixes over the original:
//   * lookup() no longer dirties the store. The old code bumped useCount and
//     re-wrote up to 4 kB of NVS every couple of seconds while a CAT-connected
//     radio was being tuned across a band, which is real flash wear for no
//     benefit. Usage statistics are now kept in RAM and flushed on a timer.
//   * Eviction no longer uses millis()/1000 as an age. That resets to zero at
//     every boot, so entries touched in the current session scored *lower* than
//     stale ones and were evicted first. A monotonic use sequence is stored
//     instead.
//
// Adds: per-band fallback and CSV import/export.
//

#include <Arduino.h>
#include <Preferences.h>

#include <vector>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

struct MemoryEntry {
  uint32_t bin = 0;
  uint8_t lMask = 0;
  uint8_t cMask = 0;
  uint8_t flags = 0;
  uint8_t antenna = 0;
  uint16_t swrX100 = 0;
  uint16_t useCount = 0;
  uint32_t lastUseSeq = 0;   // monotonic, survives reboots
} __attribute__((packed));

enum class MemHit : uint8_t { Miss, Exact, Near, Band };

struct MemLookup {
  MemHit hit = MemHit::Miss;
  RelayState state;
  uint16_t swrX100 = 0;
  uint32_t bin = 0;
};

class MemoryStore {
 public:
  void begin(Settings* settings) {
    cfg_ = settings;
    prefs_.begin("atu_mem", false);
    load();
  }

  void loop() {
    if (dirty_ && elapsed(dirtyMs_) > 2000) save();
  }

  size_t size() const { return entries_.size(); }
  const std::vector<MemoryEntry>& entries() const { return entries_; }

  // Read-only: never marks the store dirty. Call noteUse() separately when a
  // hit is actually applied to the relays.
  MemLookup lookup(uint32_t freqHz, uint8_t antenna) const {
    MemLookup out;
    if (freqHz == 0 || entries_.empty()) return out;

    uint32_t bin = binFor(freqHz);

    int bestIdx = -1;
    uint32_t bestDist = UINT32_MAX;
    for (size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].antenna != antenna) continue;
      uint32_t d = (entries_[i].bin > bin) ? (entries_[i].bin - bin) : (bin - entries_[i].bin);
      if (d < bestDist) { bestDist = d; bestIdx = static_cast<int>(i); }
      if (d == 0) break;
    }

    if (bestIdx >= 0 && bestDist <= 1) {
      fill(out, entries_[bestIdx]);
      out.hit = (bestDist == 0) ? MemHit::Exact : MemHit::Near;
      return out;
    }

    // Fall back to the nearest entry inside the same amateur band. Better than
    // nothing as a tuning seed, and never crosses a band edge.
    if (cfg_->memoryBandFallback) {
      int band = findBandIndex(freqHz);
      if (band >= 0) {
        int bandIdx = -1;
        uint32_t bandDist = UINT32_MAX;
        for (size_t i = 0; i < entries_.size(); ++i) {
          if (entries_[i].antenna != antenna) continue;
          uint32_t hz = entries_[i].bin * cfg_->memoryBinHz;
          if (findBandIndex(hz) != band) continue;
          uint32_t d = (entries_[i].bin > bin) ? (entries_[i].bin - bin) : (bin - entries_[i].bin);
          if (d < bandDist) { bandDist = d; bandIdx = static_cast<int>(i); }
        }
        if (bandIdx >= 0) {
          fill(out, entries_[bandIdx]);
          out.hit = MemHit::Band;
          return out;
        }
      }
    }

    return out;
  }

  // Records that a stored entry was actually used. Cheap; the NVS write is
  // coalesced by loop().
  void noteUse(uint32_t bin, uint8_t antenna) {
    int idx = findByBin(bin, antenna);
    if (idx < 0) return;
    MemoryEntry& e = entries_[idx];
    if (e.useCount < UINT16_MAX) ++e.useCount;
    e.lastUseSeq = ++useSeq_;
    markDirty();
  }

  void upsert(uint32_t freqHz, const RelayState& state, float swr, uint8_t antenna) {
    if (freqHz == 0) return;
    uint32_t bin = binFor(freqHz);
    int idx = findByBin(bin, antenna);
    if (idx < 0) {
      if (entries_.size() >= cfg_->maxMemoryEntries) {
        idx = pickEvictionIndex();
        if (idx < 0) return;
      } else {
        entries_.push_back(MemoryEntry{});
        idx = static_cast<int>(entries_.size() - 1);
      }
    }

    MemoryEntry& e = entries_[idx];
    e.bin = bin;
    e.lMask = state.lMask;
    e.cMask = state.cMask;
    e.flags = packFlags(state);
    e.antenna = antenna;
    e.swrX100 = static_cast<uint16_t>(clampf(swr, 1.0f, 99.99f) * 100.0f);
    if (e.useCount < UINT16_MAX) ++e.useCount;
    e.lastUseSeq = ++useSeq_;
    markDirty();
  }

  bool remove(uint32_t freqHz, uint8_t antenna) {
    int idx = findByBin(binFor(freqHz), antenna);
    if (idx < 0) return false;
    entries_.erase(entries_.begin() + idx);
    markDirty();
    return true;
  }

  void clear() {
    entries_.clear();
    useSeq_ = 0;
    prefs_.remove("table");
    prefs_.remove("seq");
    dirty_ = false;
  }

  uint32_t binFor(uint32_t freqHz) const { return freqHz / cfg_->memoryBinHz; }
  uint32_t hzForBin(uint32_t bin) const { return bin * cfg_->memoryBinHz; }

  // ---- CSV ----------------------------------------------------------------

  void exportCsv(Print& out) const {
    out.println("freqHz,lMask,cMask,topology,bypass,antenna,swr,useCount");
    for (size_t i = 0; i < entries_.size(); ++i) {
      const MemoryEntry& e = entries_[i];
      out.printf("%lu,0x%02X,0x%02X,%u,%u,%u,%.2f,%u\n",
                 static_cast<unsigned long>(hzForBin(e.bin)),
                 e.lMask, e.cMask,
                 (e.flags & FLAG_TOPOLOGY) ? 1u : 0u,
                 (e.flags & FLAG_BYPASS) ? 1u : 0u,
                 static_cast<unsigned>(e.antenna),
                 e.swrX100 / 100.0f,
                 static_cast<unsigned>(e.useCount));
    }
  }

  // Parses one CSV row. Header rows, blanks and comments are skipped silently.
  // Returns false only on a malformed data row.
  bool importCsvLine(const char* line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0' || *line == '#') return true;
    if (strncasecmp(line, "freq", 4) == 0) return true;  // header

    unsigned long hz = 0;
    int lMask = 0, cMask = 0;
    unsigned topo = 0, byp = 0, ant = 0, uses = 0;
    float swr = 0.0f;

    // %i accepts either hex (0x..) or decimal masks.
    int n = sscanf(line, "%lu,%i,%i,%u,%u,%u,%f,%u",
                   &hz, &lMask, &cMask, &topo, &byp, &ant, &swr, &uses);
    if (n < 3 || hz < 1000000UL || hz > 60000000UL) return false;

    RelayState st;
    st.lMask = static_cast<uint8_t>(lMask) & 0x7F;
    st.cMask = static_cast<uint8_t>(cMask) & 0x7F;
    st.topology = (n >= 4) && topo != 0;
    st.bypass = (n >= 5) && byp != 0;
    if (n < 7 || swr < 1.0f) swr = 1.0f;

    upsert(static_cast<uint32_t>(hz), st, swr,
           (n >= 6) ? static_cast<uint8_t>(ant) : 0);
    return true;
  }

  void flush() { if (dirty_) save(); }

 private:
  Settings* cfg_ = nullptr;
  Preferences prefs_;
  std::vector<MemoryEntry> entries_;
  bool dirty_ = false;
  uint32_t dirtyMs_ = 0;
  uint32_t useSeq_ = 0;

  void markDirty() { dirty_ = true; dirtyMs_ = millis(); }

  static void fill(MemLookup& out, const MemoryEntry& e) {
    out.state.lMask = e.lMask;
    out.state.cMask = e.cMask;
    unpackFlags(e.flags, out.state);
    out.swrX100 = e.swrX100;
    out.bin = e.bin;
  }

  int findByBin(uint32_t bin, uint8_t antenna) const {
    for (size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].bin == bin && entries_[i].antenna == antenna) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  // Evict the least valuable entry: rarely used first, and among equally used
  // entries the one least recently applied.
  int pickEvictionIndex() const {
    if (entries_.empty()) return -1;
    int best = 0;
    uint64_t minScore = UINT64_MAX;
    for (size_t i = 0; i < entries_.size(); ++i) {
      uint64_t score = static_cast<uint64_t>(entries_[i].useCount) * 1000000ULL +
                       entries_[i].lastUseSeq;
      if (score < minScore) { minScore = score; best = static_cast<int>(i); }
    }
    return best;
  }

  void load() {
    entries_.clear();
    useSeq_ = prefs_.getUInt("seq", 0);
    size_t len = prefs_.getBytesLength("table");
    if (len == 0 || (len % sizeof(MemoryEntry)) != 0) return;
    size_t count = len / sizeof(MemoryEntry);
    if (count > cfg_->maxMemoryEntries) count = cfg_->maxMemoryEntries;
    entries_.resize(len / sizeof(MemoryEntry));
    prefs_.getBytes("table", entries_.data(), len);
    entries_.resize(count);

    // Repair anything the persisted blob should not contain.
    for (size_t i = 0; i < entries_.size(); ++i) {
      entries_[i].lMask &= 0x7F;
      entries_[i].cMask &= 0x7F;
      if (entries_[i].lastUseSeq > useSeq_) useSeq_ = entries_[i].lastUseSeq;
    }
  }

  void save() {
    if (entries_.empty()) {
      prefs_.remove("table");
    } else {
      prefs_.putBytes("table", entries_.data(), entries_.size() * sizeof(MemoryEntry));
    }
    prefs_.putUInt("seq", useSeq_);
    dirty_ = false;
  }
};

}  // namespace atu
