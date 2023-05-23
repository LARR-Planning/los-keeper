//
// Created by junbs on 2023-05-22.
//

#ifndef LOS_KEEPER_STORE_H
#define LOS_KEEPER_STORE_H

namespace los_keeper {
enum PlanningResult {
  // TODO(Lee): consider other result
  kSuccess,
  kFailure
};

struct State {
  bool is_receiving_data{false};
  PlanningResult planning_result;
};

enum Action { kStart, kPause, kReset, kUpdate };

} // namespace los_keeper

#endif // LOS_KEEPER_STORE_H
