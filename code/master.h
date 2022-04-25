#ifndef MASTER_H__
#define MASTER_H__

#include "../sim/RobotInterface.h"

using namespace std;

enum mode_t{manual,planner,autonomous};

class master {
public:
  // initialize the simulator
  master();

  // add your state machine to this function
  void Run();

private:
  shared_ptr<RobotInterface> robot;
};

#endif // MASTER_H__