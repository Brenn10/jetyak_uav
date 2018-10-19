#include "jetyak_uav_utils/behaviors.h"

void behaviors::takeoffBehavior() {
  if(!propellorsRunning) {
    dji_sdk::DroneArmControl srv;
    srv.request.arm=1;
    armSrv_.call(srv);
    if(srv.response.result) {
      ROS_WARN("ARMS ENABLED");
      behaviorChanged_=true;
      currentMode_=Mode::FOLLOW;
      propellorsRunning=true;
    } else {
      ROS_WARN("FAILED TO ENABLE ARMS");
    }
  }
  else {
    ROS_WARN("ARMS ALREADY GOING");
    currentMode_=Mode::FOLLOW;
  }
}

void behaviors::followBehavior() {
  ROS_WARN("ENTER FOLLOW");
  /*if(!propellorsRunning) {
    ROS_ERROR("%s","Propellors not running, unable to follow");
  }
  else */
  if(behaviorChanged_) {
    ROS_WARN("BEHAVIOR CHANGED");
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();
    ROS_WARN("PID RESET");
    xpid_->updateParams(follow_.kp.x ,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y ,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z ,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w ,follow_.ki.w,follow_.kd.w);
    ROS_WARN("PID PARAMS RESET");
    
    behaviorChanged_=false;

  } else {
    ROS_WARN("FOLLOWING");
    // line up with pad
    xpid_->update(follow_.follow_pose.x-actualPose_.x,actualPose_.t);
    ypid_->update(follow_.follow_pose.y-actualPose_.y,actualPose_.t);
    zpid_->update(follow_.follow_pose.z-actualPose_.z,actualPose_.t);

    //point at tag
    wpid_->update(follow_.follow_pose.w-actualPose_.w,actualPose_.t);

    ROS_WARN("CONTROLLERS UPDATED");
/*    //rotate velocities in reference to the tag
    double rotated_x;
    double rotated_y;
    bsc_common::util::rotate_vector(
      xpid_->get_signal(),ypid_->get_signal(),-actualPose_.quaternion.w,rotated_x,rotated_y);
*/
    sensor_msgs::Joy cmd;
    cmd.axes.push_back(-xpid_->get_signal());
    cmd.axes.push_back(-ypid_->get_signal());
    cmd.axes.push_back(-zpid_->get_signal());
    cmd.axes.push_back(0);
    //cmd.axes.push_back(wpid_->get_signal());
    cmd.axes.push_back(bodyVelCmdFlag_);
    cmdPub_.publish(cmd);
    ROS_WARN("COMMANDS SENT");
  }
}

void behaviors::returnBehavior() {};

void behaviors::landBehavior() {};

void behaviors::rideBehavior() {
  if(propellorsRunning) {
    dji_sdk::DroneArmControl srv;
    srv.request.arm=0;
    armSrv_.call(srv);
    propellorsRunning=srv.response.result;
    if(srv.response.result) {
      ROS_WARN("Arms deactivated");
    } else {
      ROS_WARN("Failed to deactivate arms");
    }
  }
}


void behaviors::hoverBehavior() {
  sensor_msgs::Joy cmd;
  cmd.axes.push_back(0);
  cmd.axes.push_back(0);
  cmd.axes.push_back(0);
  cmd.axes.push_back(0);
  cmd.axes.push_back(bodyVelCmdFlag_);
  cmdPub_.publish(cmd);
};
