#include "jetyak_uav_utils/behaviors.h"

void behaviors::takeoffBehavior() {
  if(!propellorsRunning) {
    dji_sdk::DroneTaskControl srv;
    srv.request.task=4;
    taskSrv_.call(srv);
    if(srv.response.result) {
      ROS_INFO("Takeoff successful");
      behaviorChanged_=true;
      currentMode_=Mode::FOLLOW;
      propellorsRunning=true;
    } else {
      ROS_WARN("Failure to Takeoff");
    }
  }
  else {
    ROS_INFO("Already up");
    behaviorChanged_=true;
    currentMode_=Mode::FOLLOW;
    propellorsRunning=true;
  }
}

void behaviors::followBehavior() {

  if(behaviorChanged_) { // if just changed
    follow_.lastSpotted=simpleTag_.t;
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

    if(follow_.lastSpotted!=simpleTag_.t) { //if time changed
      follow_.lastSpotted=simpleTag_.t;
      follow_.lostTagCounter=0;
    }
    else { //if time is same
      follow_.lostTagCounter++;

      if(follow_.lostTagCounter>3 and follow_.lostTagCounter<=20) { //if time has been same for over 3 tick
        ROS_WARN("No tag update: %i" , follow_.lostTagCounter);
        sensor_msgs::Joy cmd;
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(simpleTag_.y>0 ? .3 : -.3); //rotate CCW if lost on left, CW if right
        cmd.axes.push_back(bodyVelCmdFlag_);
        cmdPub_.publish(cmd);
        return;
      } else if (follow_.lostTagCounter>20){ // if time has been same for over 10 tick
        ROS_WARN("Tag Lost");
      	sensor_msgs::Joy cmd;
      	cmd.axes.push_back(0);
      	cmd.axes.push_back(0);
      	cmd.axes.push_back(0);
      	cmd.axes.push_back(0);
      	cmd.axes.push_back(bodyVelCmdFlag_);
      	cmdPub_.publish(cmd);
        return;
      }
    }

    // line up with pad
    xpid_->update(follow_.follow_pose.x-simpleTag_.x,simpleTag_.t);
    ypid_->update(follow_.follow_pose.y-simpleTag_.y,simpleTag_.t);
    zpid_->update(follow_.follow_pose.z-simpleTag_.z,simpleTag_.t);
    wpid_->update(follow_.follow_pose.w-simpleTag_.w,simpleTag_.t);

/*    //rotate velocities in reference to the tag
    double rotated_x;
    double rotated_y;
    bsc_common::util::rotate_vector(
      xpid_->get_signal(),ypid_->get_signal(),-simpleTag_.w,rotated_x,rotated_y);
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

void behaviors::returnBehavior() {
  /* Algorithm 2
  if tag found in last X second
    set stage to SETTLE
    if yaw~0, x~2, y~0, z~(mast height+1)
      enter follow modes
    else
      PID to the correct points

  elif first time
    Set stage to UP
    init PID

  elif stage == up
    if altitude is >gotoHeight
      set stage to Over
    else
      command 0xy and the goal height-altitude
  elif stage==over
    saturate x and y with directions
    z_cmd = gotoHeight-altitude
    if in sphere around goal
      set stage to DOWN
  elif stage==down
    find proper pid constants to maintain location more or less
    z=finalHeight-altitude
    we wont change the mode here as it should search for the tags and hopefully find one
  elif stage=settle
    Yaw or elevate or translate to recover tags
    If lost for too long, return to gps following
  */
  double east = bsc_common::util::latlondist(0,boatGPS_.longitude,0,uavGPS_.longitude);
  double north = bsc_common::util::latlondist(boatGPS_.latitude,0,uavGPS_.latitude,0);
  double heading = atan2(north,east);


  if(ros::Time::now().toSec()-tagPose_.header.stamp.toSec()<return_.tagTime){
    /*
      set stage to SETTLE
      if yaw~0, x~2, y~0, z~(mast height+1)
        enter follow modes
      else
        PID to the correct points
    */
    return_.stage=return_.SETTLE;
    if(pow(follow_.follow_pose.x-simpleTag_.x,2)<return_.settleRadiusSquared and
       pow(follow_.follow_pose.x-simpleTag_.y,2)<return_.settleRadiusSquared and
       pow(follow_.follow_pose.w-simpleTag_.w,2)<return_.settleRadiusSquared)
    {
      currentMode_=Mode::FOLLOW;
    } else {// line up with pad
      xpid_->update(follow_.follow_pose.x-simpleTag_.x,simpleTag_.t);
      ypid_->update(follow_.follow_pose.y-simpleTag_.y,simpleTag_.t);
      zpid_->update(return_.finalHeight-simpleTag_.z,simpleTag_.t);
      wpid_->update(follow_.follow_pose.w-simpleTag_.w,simpleTag_.t);


      sensor_msgs::Joy cmd;
      cmd.axes.push_back(-xpid_->get_signal());
      cmd.axes.push_back(-ypid_->get_signal());
      cmd.axes.push_back(-zpid_->get_signal());
      cmd.axes.push_back(-wpid_->get_signal());
      cmd.axes.push_back(bodyVelCmdFlag_);
      cmdPub_.publish(cmd);
    }
  }

  else if(behaviorChanged_) {
    /*
      set stage to up
      set behavior changed to false
      init pid
    */
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();
    xpid_->updateParams(follow_.kp.x ,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y ,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z ,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w ,follow_.ki.w,follow_.kd.w);
    behaviorChanged_=false;
    return_.stage=return_.UP;

  }

  else if(return_.stage==return_.UP) {
    /*if altitude is >gotoHeight
        set stage to Over
      else
        command 0xy and the goal height-altitude
    */
    if(uavGPS_.altitude-boatGPS_.altitude>return_.gotoHeight) {
      return_.stage=return_.OVER;
    }
    else {
      double z_correction=follow_.kp.z*(
        return_.gotoHeight-(uavGPS_.altitude-boatGPS_.altitude)
      );

      sensor_msgs::Joy cmd;
      cmd.axes.push_back(0);
      cmd.axes.push_back(0);
      cmd.axes.push_back(z_correction);
      cmd.axes.push_back(heading);
      cmd.axes.push_back(worldPosCmdFlag_);
      cmdPub_.publish(cmd);
    }
  }
  else if(return_.stage==return_.OVER) {
    /*
      saturate x and y with directions
      z_cmd = gotoHeight-altitude
      if in sphere around goal
        set stage to DOWN
    */
    if(bsc_common::util::latlondist(uavGPS_.latitude,uavGPS_.longitude,
        boatGPS_.latitude,boatGPS_.longitude) < return_.downRadius)
    {
      return_.stage=return_.DOWN;
    }
    else {
      double z_correction=follow_.kp.z*(
        return_.gotoHeight+.1-(uavGPS_.altitude-boatGPS_.altitude)
      );

      sensor_msgs::Joy cmd;
      cmd.axes.push_back(east);
      cmd.axes.push_back(north);
      cmd.axes.push_back(z_correction);
      cmd.axes.push_back(heading);
      cmd.axes.push_back(worldPosCmdFlag_);
      cmdPub_.publish(cmd);
    }
  }
  else if(return_.stage=return_.DOWN) {
  /*
    find proper pid constants to maintain location more or less
    z=finalHeight-altitude
    we wont change the mode here as it should search for the tags and hopefully find one
  */
    double z_correction=follow_.kp.z*(
      return_.finalHeight-(uavGPS_.altitude-boatGPS_.altitude)
    );

    sensor_msgs::Joy cmd;
    cmd.axes.push_back(east);
    cmd.axes.push_back(north);
    cmd.axes.push_back(z_correction);
    cmd.axes.push_back(heading);
    cmd.axes.push_back(worldPosCmdFlag_);
    cmdPub_.publish(cmd);
  }
  else {
    ROS_ERROR_ONCE("BAD CONDITIONALS, DEFAULTING TO HOVER");
    currentMode_=Mode::HOVER;
  }
};

void behaviors::landBehavior() {
  if(behaviorChanged_) { // if just changed
    land_.lastSpotted=simpleTag_.t;
    land_.lostTagCounter=0;
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();
    xpid_->updateParams(land_.kp.x ,land_.ki.x,land_.kd.x);
    ypid_->updateParams(land_.kp.y ,land_.ki.y,land_.kd.y);
    zpid_->updateParams(land_.kp.z ,land_.ki.z,land_.kd.z);
    wpid_->updateParams(land_.kp.w ,land_.ki.w,land_.kd.w);
    behaviorChanged_=false;

  } else { // DO the loop

    if(follow_.lastSpotted!=simpleTag_.t) { //if time changed

      // If pose is within a cylinder of radius .1 and height .1
      if((land_.land_pose.z-simpleTag_.z)<.1 and
        pow(land_.land_pose.x-simpleTag_.x,2)+
        pow(land_.land_pose.y-simpleTag_.y,2)<pow(.1,2))
      {
        dji_sdk::DroneTaskControl srv;
        srv.request.task=6;
        taskSrv_.call(srv);
        if(srv.response.result)
        {
          currentMode_=Mode::RIDE;
          return;
        }
      }

      land_.lastSpotted=simpleTag_.t;
      land_.lostTagCounter=0;
    }
    else { //if time is same
      follow_.lostTagCounter++;

      if(follow_.lostTagCounter>3 and follow_.lostTagCounter<=20) { //if time has been same for over 3 tick
        ROS_WARN("No tag update: %i" , follow_.lostTagCounter);
        sensor_msgs::Joy cmd;
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(.5); //If tag lost, fly up a bit
        cmd.axes.push_back(0);
        cmd.axes.push_back(bodyVelCmdFlag_);
        cmdPub_.publish(cmd);
        return;
      } else if (follow_.lostTagCounter>20){ // if time has been same for over 10 tick
        ROS_WARN("Tag Lost");
        sensor_msgs::Joy cmd;
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(0);
        cmd.axes.push_back(bodyVelCmdFlag_);
        cmdPub_.publish(cmd);
        return;
      }
    }

    // line up with pad
    xpid_->update(land_.land_pose.x-simpleTag_.x,simpleTag_.t);
    ypid_->update(land_.land_pose.y-simpleTag_.y,simpleTag_.t);
    zpid_->update(land_.land_pose.z-simpleTag_.z,simpleTag_.t);
    wpid_->update(land_.land_pose.w-simpleTag_.w,simpleTag_.t);

  /*    //rotate velocities in reference to the tag
    double rotated_x;
    double rotated_y;
    bsc_common::util::rotate_vector(
      xpid_->get_signal(),ypid_->get_signal(),-simpleTag_.w,rotated_x,rotated_y);
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
};

void behaviors::rideBehavior() {
  if(propellorsRunning) {
    dji_sdk::DroneTaskControl srv;
    srv.request.task=6;
    taskSrv_.call(srv);
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
