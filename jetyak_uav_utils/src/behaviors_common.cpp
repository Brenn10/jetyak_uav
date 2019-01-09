#include "jetyak_uav_utils/behaviors.h"

void Behaviors::resetPID() {
  xpid_->reset();
  ypid_->reset();
  zpid_->reset();
  wpid_->reset();
}

void Behaviors::setPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki,
                       bsc_common::pose4d_t &kd) {
  xpid_->updateParams(kp.x, ki.x, kd.x);
  ypid_->updateParams(kp.y, ki.y, kd.y);
  zpid_->updateParams(kp.z, ki.z, kd.z);
  wpid_->updateParams(kp.w, ki.w, kd.w);
}