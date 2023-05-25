#ifndef HEADER_STORE
#define HEADER_STORE

#include "los_keeper/type_manager/type_manager.h"

namespace los_keeper {
namespace store {

struct State {
  StatePoly planning_output;
  bool is_all_data_received{false};
  bool is_planning_valid{false};
};

enum Action {
  kUpdateMonitor,
  kInitialize,
  kReplan,
};

Action DecideAction(const State &state);
} // namespace store
} // namespace los_keeper

#endif /* HEADER_STORE */
