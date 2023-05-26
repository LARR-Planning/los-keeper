#ifndef HEADER_STORE
#define HEADER_STORE

#include "los_keeper/type_manager/type_manager.h"

namespace los_keeper {
typedef StatePoly PlanningOutput;

namespace store {

struct State {
  bool is_initialized{false};
  bool is_planning_safe{false};
  bool is_planning_visible{false};
  bool is_planning_expired{false};
  bool is_currently_safe{true};
};

enum Action {
  kInitialize,
  kReplan,
  kStop,
};

Action DecideAction(const State &state);
} // namespace store
} // namespace los_keeper

#endif /* HEADER_STORE */
