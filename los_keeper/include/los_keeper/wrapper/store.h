#ifndef HEADER_STORE
#define HEADER_STORE

#include "los_keeper/type_manager/type_manager.h"

namespace los_keeper {

namespace store {

struct State {
  bool is_activated{false};
  bool is_data_received{false};
  // TODO(@): set to false when safe / visible checks work
  bool is_planning_safe{true};
  bool is_planning_visible{true};
  bool is_planning_expired{false};
  bool is_currently_safe{true};
};

enum Action {
  kIdle,
  kReplan,
  kStop,
};

Action DecideAction(const State &state);
} // namespace store
} // namespace los_keeper

#endif /* HEADER_STORE */
