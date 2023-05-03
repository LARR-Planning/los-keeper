#include "Wrapper.hpp"

std::string los_keeper::Wrapper::GetName() const {
  return name_ + ": " + chasing_planner_.GetName() + " " +
         target_manager_.GetName() + " " + obstacle_manager_.GetName();
}