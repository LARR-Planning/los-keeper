#include "los_keeper/wrapper/store.h"

namespace los_keeper {
namespace store {
Action DecideActionForCallback(const State &state) {
  if (!state.is_planning_valid)
    return Action::kReplan;
}

} // namespace store

} // namespace los_keeper