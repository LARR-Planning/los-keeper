#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

bool Wrapper::Plan() const {
  return target_manager_.GetName() == "TargetManager" &&
         obstacle_manager_.GetName() == "ObstacleManager";
}

void Wrapper::SetLongString(const std::string &long_string) {
  long_string_.clear();

  for (char c : long_string) {
    long_string_.push_back(c);
    long_string_ += ' ';

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
};
void Wrapper::SetShortString(const std::string &short_string) {
  short_string_.clear();

  for (char c : short_string) {
    short_string_.push_back(c);
    short_string_ += ' ';

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
};

std::string Wrapper::GetConcatString() const {
  return long_string_ + " and " + short_string_;
}
