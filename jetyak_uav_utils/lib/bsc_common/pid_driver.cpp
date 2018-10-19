#include "include/pid.h"
#include <iostream>

int main() {
  bsc_common::PID *pid = new bsc_common::PID();
  pid->reset();
  return 0;
}
