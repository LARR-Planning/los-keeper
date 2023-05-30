#include "los_keeper/wrapper/store.h"

namespace los_keeper {
namespace store {
Action DecideAction(const State &state) {
  if (!state.is_activated)
    return Action::kIdle;
  if (!state.is_currently_safe) {
    return Action::kStop;
  }
  if (!state.is_planning_expired ||
      (!state.is_planning_safe || !state.is_planning_visible))
    return Action::kReplan;
  return Action::kIdle;
}
} // namespace store
} // namespace los_keeper
