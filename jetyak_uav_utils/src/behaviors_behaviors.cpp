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
  /*if(!propellorsRunning) {
    ROS_ERROR("%s","Propellors not running, unable to follow");
  }
  else */
  if(behaviorChanged_) { // if just changed
    follow_.lastSpotted=actualPose_.t;
    follow_.lostTagCounter=0;
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();
    xpid_->updateParams(follow_.kp.x ,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y ,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z ,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w ,follow_.ki.w,follow_.kd.w);
    behaviorChanged_=false;

  } else { // DO the loop

    if(follow_.lastSpotted!=actualPose_.t) { //if time changed
      follow_.lastSpotted=actualPose_.t;
      follow_.lostTagCounter=0;
    }
    else { //if time is same
      follow_.lostTagCounter++;

      if(follow_.lostTagCounter>3 and follow_.lostTagCounter<=10) { //if time has been same for over 3 tick
        sensor_msgs::Joy cmd;
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(y>0 ? .1 : -.1); //rotate CCW if lost on left, CW if right
        cmd.axes.push_back(bodyVelCmdFlag_);
        cmdPub_.publish(cmd);
        return;
      } else if (follow_.lostTagCounter>10){ // if time has been same for over 10 tick
        behaviorChanged_=true;
        currentMode_=Mode::HOVER;
        return;
      }
    }

    // line up with pad
    xpid_->update(follow_.follow_pose.x-actualPose_.x,actualPose_.t);
    ypid_->update(follow_.follow_pose.y-actualPose_.y,actualPose_.t);
    zpid_->update(follow_.follow_pose.z-actualPose_.z,actualPose_.t);
    wpid_->update(follow_.follow_pose.w-actualPose_.w,actualPose_.t);

/*    //rotate velocities in reference to the tag
    double rotated_x;
    double rotated_y;
    bsc_common::util::rotate_vector(
      xpid_->get_signal(),ypid_->get_signal(),-actualPose_.w,rotated_x,rotated_y);
*/
    ROS_INFO("sig x: %1.2f, y:%1.2f, z: %1.2f, yaw: %1.3f",
      -xpid_->get_signal(),
      -ypid_->get_signal(),
      -zpid_->get_signal(),
      -wpid_->get_signal()
    );
    sensor_msgs::Joy cmd;
    cmd.axes.push_back(-xpid_->get_signal());
    cmd.axes.push_back(-ypid_->get_signal());
    cmd.axes.push_back(-zpid_->get_signal());
    cmd.axes.push_back(-wpid_->get_signal());
    cmd.axes.push_back(bodyVelCmdFlag_);
    cmdPub_.publish(cmd);


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
