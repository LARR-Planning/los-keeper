#ifndef HEADER_TARGETMANAGER
#define HEADER_TARGETMANAGER
#include <string>

namespace los_keeper {

class TargetManager {
private:
  std::string name_{"TargetManager"};

public:
  std::string GetName() const;
};
} // namespace los_keeper

#endif /* HEADER_TARGETMANAGER */
