#pragma once
//
// Relay bank state. Free of Arduino headers so the tuning maths can be built
// and tested on a desktop machine (see test/).
//

#include <cstdint>

namespace atu {

struct RelayState {
  uint8_t lMask = 0;
  uint8_t cMask = 0;
  bool topology = false;  // false = Lo-Z, true = Hi-Z
  bool bypass = false;

  bool operator==(const RelayState& o) const {
    return lMask == o.lMask && cMask == o.cMask && topology == o.topology &&
           bypass == o.bypass;
  }
  bool operator!=(const RelayState& o) const { return !(*this == o); }
};

enum RelayFlag : uint8_t {
  FLAG_TOPOLOGY = 0x01,
  FLAG_BYPASS = 0x02,
};

inline uint8_t packFlags(const RelayState& s) {
  return static_cast<uint8_t>((s.topology ? FLAG_TOPOLOGY : 0) |
                              (s.bypass ? FLAG_BYPASS : 0));
}

inline void unpackFlags(uint8_t flags, RelayState& s) {
  s.topology = (flags & FLAG_TOPOLOGY) != 0;
  s.bypass = (flags & FLAG_BYPASS) != 0;
}

}  // namespace atu
