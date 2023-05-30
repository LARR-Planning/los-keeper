#include "los_keeper/wrapper/store.h"

namespace los_keeper {
namespace store {
Action DecideAction(const State &state) {
  if (!state.is_currently_safe) {
    return Action::kStop;
  } else if (!state.is_data_received) {
    return Action::kInitialize;
  } else if (!state.is_planning_expired ||
             (!state.is_planning_safe || !state.is_planning_visible))
    return Action::kReplan;
}
} // namespace store
} // namespace los_keeper
