#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh):
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
 {
  arTagSub_ = nh.subscribe("/ar_pose_marker",1,&behaviors::arTagCallback, this);
  cmdPub_ = nh.advertise<sensor_msgs::Joy>("jetyak_uav_utils/behaviorCmd",1);
  modeService_ = nh.advertiseService("jetyak_uav_utils/mode", &behaviors::modeCallback,this);
  taskSrv_ = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");


  currentMode_ = 0;

}

behaviors::~behaviors() {}

void behaviors::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  // If tags detected
  if(!msg->markers.empty()){
    //get time of detection
    tagLastSeen_=ros::Time::now().toSec();

    // get the transform from the tag to the uav
    geometry_msgs::Pose pose_from_tag;
    bsc_common::util::inverse_pose(msg->markers[0].pose.pose,pose_from_tag);

    //convert the orientation to rpy
    const geometry_msgs::Quaternion* orientation = const_cast<const geometry_msgs::Quaternion*>(&pose_from_tag.orientation);
    geometry_msgs::Vector3* state = new geometry_msgs::Vector3();
    bsc_common::util::rpy_from_quat(orientation,state);//convert quat to rpy

    //save pose
    uavPose_.x=pose_from_tag.position.x;
    uavPose_.y=pose_from_tag.position.y;
    uavPose_.z=pose_from_tag.position.z;
    uavPose_.w=state->z;

    delete state;


    /***************************************************************************
    * Behavior actions based on ar_tag
    ***************************************************************************/
    switch(this->currentMode_)
    {
      // LAND BEHAVIOR HERE
      case jetyak_uav_utils::Mode::Request::LAND:
      {
        /* If close enough, request a landing from sdk
         * If first time, reset controller and do set constants
         * else decrease distance and update controller
         */
        bool closeEnough = uavPose_.z<.1;


        if(closeEnough)
        {
          dji_sdk::DroneTaskControl srv;
          srv.request.task=6;
          taskSrv_.call(srv);
          if(srv.response.result)
          {
            this->currentMode_=jetyak_uav_utils::Mode::Request::LANDED;
          }
          else
          {
            ROS_WARN("Failed to land, hovering");
            this->currentMode_=jetyak_uav_utils::Mode::Request::HOVER;
          }
        }
        else if(behaviorChanged)
        {
          land_.currGoalHeight=land_.START_HEIGHT;

          behaviorChanged=false;

          // update PID Constants
          xpid_->updateParams(land_.kp.x,land_.ki.x,land_.kd.x);
          ypid_->updateParams(land_.kp.y,land_.ki.y,land_.kd.y);
          zpid_->updateParams(land_.kp.z,land_.ki.z,land_.kd.z);
          wpid_->updateParams(land_.kp.w,land_.ki.w,land_.kd.w);

          // reset PIDs
          xpid_->reset();
          ypid_->reset();
          zpid_->reset();
          wpid_->reset();

        }
        else
        {
          //collapse height
          land_.currGoalHeight*=land_.collapseRatio;

          //update controllers
          xpid_->update(-uavPose_.x);
          ypid_->update(-uavPose_.y);
          zpid_->update(land_.currGoalHeight-uavPose_.z);
          wpid_->update(-uavPose_.w);

          //rotate velocities in reference to the uav
          double rotated_x;
          double rotated_y;
          bsc_common::util::rotate_vector(
            xpid_->get_signal(),ypid_->get_signal(),-uavPose_.w,rotated_x,rotated_y);

          // set cmd things and publish
          sensor_msgs::Joy cmd;
          cmd.axes.push_back(rotated_x);
          cmd.axes.push_back(rotated_y);
          cmd.axes.push_back(wpid_->get_signal());
          cmd.axes.push_back(zpid_->get_signal());
          cmdPub_.publish(cmd);
        }

        break;
      }
      // FOLLOW BEHAVIOR HERE
      case jetyak_uav_utils::Mode::Request::FOLLOW: {
        if(behaviorChanged)
        {
          behaviorChanged=false;

          // update PID Constants
          xpid_->updateParams(follow_.kp.x,follow_.ki.x,follow_.kd.x);
          ypid_->updateParams(follow_.kp.y,follow_.ki.y,follow_.kd.y);
          zpid_->updateParams(follow_.kp.z,follow_.ki.z,follow_.kd.z);
          wpid_->updateParams(follow_.kp.w,follow_.ki.w,follow_.kd.w);

          // reset PIDs
          xpid_->reset();
          ypid_->reset();
          zpid_->reset();
          wpid_->reset();
        }
        else
        {
          //update controllers
          xpid_->update(follow_.follow_pose.x-uavPose_.x);
          ypid_->update(follow_.follow_pose.y-uavPose_.y);
          zpid_->update(follow_.follow_pose.z-uavPose_.z);
          wpid_->update(bsc_common::util::ang_dist(follow_.follow_pose.w,uavPose_.w));//TODO: Check sign

          //rotate velocities in reference to the uav
          double rotated_x;
          double rotated_y;
          bsc_common::util::rotate_vector(
            xpid_->get_signal(),ypid_->get_signal(),-uavPose_.w,rotated_x,rotated_y);

          // set cmd things and publish
          sensor_msgs::Joy cmd;
          cmd.axes.push_back(rotated_x);
          cmd.axes.push_back(rotated_y);
          cmd.axes.push_back(wpid_->get_signal());
          cmd.axes.push_back(zpid_->get_signal());
          cmdPub_.publish(cmd);
        }
        break;
      }
      // RETURN BEHVAIOR HERE
      case jetyak_uav_utils::Mode::Request::RETURN: {

        break;
      }
      case jetyak_uav_utils::Mode::Request::HOVER: {

        break;
      }
    }




  }
}

bool behaviors::modeCallback(jetyak_uav_utils::Mode::Request  &req,
                              jetyak_uav_utils::Mode::Response &res) {

  this->currentMode_ = req.mode;

  switch(this->currentMode_)
  {
    case req.TAKEOFF: {
      dji_sdk::DroneTaskControl srv;
      srv.request.task=4;
      taskSrv_.call(srv);
      res.success=srv.response.result;
      if(res.success)
      {
        this->currentMode_=req.FOLLOW;
      }
      else
      {
        this->currentMode_=req.HOVER;
      }
    }
    case req.LAND: {}
    case req.FOLLOW: {}
    case req.LEAVE: {}
    case req.RETURN: {
      res.success=true;
      break;
    }

  }
  behaviorChanged=true;
  return res.success;
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_behaviors");
  ros::NodeHandle nh;
  behaviors uav_behaviors(nh);
  ros::spin();
  // ros::Rate rate(10);
  //
  // while(ros::ok())
  // {
  //   ros::spinOnce();
  //
  //   uav_behaviors.publishCommand();
  //
  //   rate.sleep();
  // }
  return 0;
}
