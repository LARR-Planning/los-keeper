#include "los_server/los_server.h"
#include "los_server.h"

using namespace los_keeper;
bool LosServer::Update() { return wrapper_.Plan(); }

LosServer::LosServer() {}
