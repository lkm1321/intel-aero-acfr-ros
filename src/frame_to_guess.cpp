#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <rtabmap_ros/OdomInfo.h> 

using namespace ros; 

class VIOBootStrap {
 private:
  ros::NodeHandle _nh;
  ros::Subscriber _imu_sub; 
  ros::Subscriber _last_pose_sub; 

  tf::TransformBroadcaster _br; 	   
  tf::Transform _transform; 
 
 public:
  VIOBootStrap () {
    _imu_sub = _nh.subscribe("/mavros/imu/data", 10, &VIOBootStrap::imuCallback, this); 
    _last_pose_sub = _nh.subscribe("/mavros/local_position/pose", 10, &VIOBootStrap::poseCallback, this); 
    _transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    _transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) ); 
  }  
  
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg){
    //  tf::Transform transform;
    tf::Quaternion quat = tf::Quaternion(imuMsg->orientation.x, 
					 imuMsg->orientation.y, 
					 imuMsg->orientation.z,
					 imuMsg->orientation.w); 
    // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) ); 
    _transform.setRotation(quat); 
    _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "map", "guess_frame")); 
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
    _transform.setOrigin( tf::Vector3(poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z) );     
    _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "map", "guess_frame"));  
  } 	

};

int main(int argc, char** argv){
	
	// Initialize
	ros::init(argc, argv, "vio_bootstrap"); 
	VIOBootStrap vbs;  
	// Process callbacks. 
	ros::spin();
	return 0;

}

