#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

void imageCallback(const sensor_msgs::ImageConstPtr& depth_image){
	SimpleBlobDetector:Params params; 
	
	// Min and max threshold.
	params.minThreshold = 200; // mm 
	params.maxThreshold = 2000; // mm
	params.filterByArea = true; 
	params.minArea = 250; // pixel^2

	cv_bridge::CvImagePtr depth_image_cv = toCvCopy(depth);  

}

int main(int argc, char** argv){
	
	image_transport::ImageTransport it; 
	image_transport::Subscriber image_sub; 
	image_transport::Publisher image_pub; 
	
	// Initialize
	ros::init(argc, argv, "camera_fcu_tf"); 
	
	// 60Hz update. 
	ros::Rate r(60); 
	
	
	while(ros::ok()){
		
		ros::spinOnce(); 
		r.sleep(); 
	}
	return 0;
}
