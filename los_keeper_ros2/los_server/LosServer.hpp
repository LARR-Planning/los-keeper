#ifndef HEADER_LOSSERVER
#define HEADER_LOSSERVER
#include <los_keeper/wrapper/Wrapper.hpp>
namespace los_keeper {
class LosServer {
private:
  Wrapper wrapper_;

public:
  LosServer() { wrapper_.GetName(); }
};

} // namespace los_keeper
#endif /* HEADER_LOSSERVER */
