#include "include/pid.h"
#include <iostream>

int main() {
  bsc_common::PID pid = bsc_common::PID();
  pid.reset();
  return 0;
}
