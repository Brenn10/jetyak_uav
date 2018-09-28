#include "jetyak_uav_utils/gimbal_tag.h"

#include "tf/transform_datatypes.h"

gimbal_tag::gimbal_tag(ros::NodeHandle& nh)
{
	// Subscribe to topics
	tagPoseSub = nh.subscribe("ar_pose_marker", 10, &gimbal_tag::tagCallback, this);
	gimbalAngleSub = nh.subscribe("dji_sdk/gimbal_angle", 10, &gimbal_tag::gimbalCallback, this);
	vehicleAttiSub = nh.subscribe("dji_sdk/attitude", 10, &gimbal_tag::attitudeCallback, this);

	// Set up publisher
	tagBodyPosePub = nh.advertise<geometry_msgs::PoseStamped>("tag_pose", 10);

	tagFound = false;

	// Initialize the constant offset between Gimbal and Vehicle orientation
	// !!! The constant offset is measured on Matrice 100 and Z3 !!!
	qConstant = tf::Quaternion(0.0, 0.0, -0.707, 0.707);
	qConstant.normalize();
	qCamera2Gimbal = tf::Quaternion(-0.5, 0.5, -0.5, 0.5);
}

void gimbal_tag::publishTagPose()
{
	if(tagFound)
	{
		// The vehicle quaternion is the rotation from body_FLU to ground_ENU
		// The Gimbal rotation is from the ground_ENU to Gimbal body

		// Calculate offset quaternion
		qOffset = qVehicle.inverse() * qGimbal.inverse();
		qOffset.normalize();

		// Apply rotation to go from gimbal frame to body frame
		tf::Quaternion qTagBody = qOffset*qTag;
		qTagBody.normalize();
		tf::Quaternion positonTagBody = qOffset*posTag*qOffset.inverse();
		geometry_msgs::PoseStamped tagPoseBody;

		// Get time
		ros::Time time = ros::Time::now();

		// Update header
		tagPoseBody.header.stamp = time;
		tagPoseBody.header.frame_id = "body_FLU";

		tagPoseBody.pose.position.x = positonTagBody[0];
		tagPoseBody.pose.position.y = positonTagBody[1];
		tagPoseBody.pose.position.z = positonTagBody[2];

		tf::quaternionTFToMsg(qTagBody, tagPoseBody.pose.orientation);

		tagBodyPosePub.publish(tagPoseBody);
	}
}

// Callbacks
void gimbal_tag::tagCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
	if(!msg.markers.empty())
	{
		// Update Tag quaternion
		tf::quaternionMsgToTF(msg.markers[0].pose.pose.orientation, qTag);
		qTag.normalize();

		// Update Tag position as quaternion
		posTag[0] = msg.markers[0].pose.pose.position.x;
		posTag[1] = msg.markers[0].pose.pose.position.y;
		posTag[2] = msg.markers[0].pose.pose.position.z;
		posTag[3] = 0;

		// Go from Camera frame to Gimbal frame
		qTag = qCamera2Gimbal*qTag;
		posTag = qCamera2Gimbal*posTag*qCamera2Gimbal.inverse();

		tagFound = true;
	}
	else
		tagFound = false;
}

void gimbal_tag::gimbalCallback(const geometry_msgs::Vector3Stamped& msg)
{
	// Update gimbal quaternion
	qGimbal = tf::createQuaternionFromRPY(
		DEG2RAD(msg.vector.x), DEG2RAD(msg.vector.y), DEG2RAD(msg.vector.z));
	// Remove the constant offset
	qGimbal = qConstant*qGimbal;
	qGimbal.normalize();
}

void gimbal_tag::attitudeCallback(const geometry_msgs::QuaternionStamped& msg)
{
	// Update Vehicle quaternion
	tf::quaternionMsgToTF(msg.quaternion, qVehicle);
	qVehicle.normalize();
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gimbal_test");
  ros::NodeHandle nh;

  gimbal_tag tagTracker(nh);

  ros::Rate rate(10);

  while(ros::ok())
  {
  	ros::spinOnce();
  	tagTracker.publishTagPose();
  	rate.sleep();
  }

  return 0;
}
