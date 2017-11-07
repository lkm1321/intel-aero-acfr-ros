#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/features2d.hpp>

class RosBlobDetector {
	ros::NodeHandle _nh; // Node handle. 
	image_transport::ImageTransport _it; // Needed for iamge transport.
	image_transport::Subscriber _image_sub; // Subscription handle.  
	image_transport::Publisher _image_pub;  // Publication handle. 
	cv::Ptr<cv::SimpleBlobDetector> _detector; 
	std::vector<cv::KeyPoint> keypoints; 
public:
	RosBlobDetector(): _it(_nh){
		_image_sub = _it.subscribe("/camera/depth/image_raw", 10, &RosBlobDetector::imageCallback, this);
		_image_pub = _it.advertise("/blob_detector/result", 10); 
		cv::SimpleBlobDetector::Params params; 
		// Min and max threshold.
		params.minThreshold = 100; // mm 
		params.maxThreshold = 2000; // mm
		params.filterByArea = true; 
		params.minArea = 250; // pixel^2
		_detector = cv::SimpleBlobDetector::create(params); 
	}
	
	void imageCallback(const sensor_msgs::ImageConstPtr& depth_image){

		// Convert to OpenCV image
		cv_bridge::CvImagePtr depth_image_cv_ptr; 

		try{
			cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::MONO16);  
		} catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception %s at %s, line %d", e.what(), __FILE__, __LINE__); 
			return; 
		}

		_detector->detect(depth_image_cv_ptr->image, keypoints);
		cv::drawKeypoints( depth_image_cv_ptr->image, depth_image_cv_ptr->image, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

		_image_pub.publish(depth_image_cv_ptr->toImageMsg());
	}

};
int main(int argc, char** argv){
	
	// Initialize
	ros::init(argc, argv, "ros_blob_detector"); 
	RosBlobDetector rbd; 
	// Process callbacks. 
	ros::spin();
	return 0;
}
