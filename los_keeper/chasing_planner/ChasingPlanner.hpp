#ifndef HEADER_CHASINGPLANNER
#define HEADER_CHASINGPLANNER

#include <string>
namespace los_keeper {
class ChasingPlanner {
private:
  std::string name_{"ChasingPlanner"};

public:
  std::string GetName() const;
};

} // namespace los_keeper

#endif /* HEADER_CHASINGPLANNER */
