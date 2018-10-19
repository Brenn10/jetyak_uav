#include "include/pid.h"
#include <iostream>

int main() {
  bsc_common::PID* pid = new bsc_common::PID(1,0,0);
  for(double i =0; i < 1; i+=.01) {
    pid->update(1.0,i);
    std::cout << pid->get_signal() << std::endl;
  }
  return 0;
}
