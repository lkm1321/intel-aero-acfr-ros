#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/features2d.hpp>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


class RosBlobDetector {

private:
	ros::NodeHandle _nh; // Node handle. 
	image_transport::ImageTransport _it; // Needed for iamge transport.
	image_transport::Subscriber _image_sub; // Subscription handle.  
	image_transport::Publisher _image_pub;  // Publication handle. 
	ros::Publisher _range_pub;  
	float _oldRange; 
	mavros_msgs::State _current_state; 
	ros::Subscriber _state_sub; 
	ros::ServiceClient _set_mode_cli; 
	
public:
	RosBlobDetector(): _it(_nh){
		_image_sub = _it.subscribe("/camera/depth/image_raw", 10, &RosBlobDetector::imageCallback, this);
		_image_pub = _it.advertise("/blob_detector/result", 10); 
		_range_pub = _nh.advertise<sensor_msgs::Range>("/blob_detector/range", 50);
		_state_sub = _nh.subscribe("/mavros/state", 10, &RosBlobDetector::stateCallback, this); 
		_set_mode_cli = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); 
	}

	void stateCallback(const mavros_msgs::State::ConstPtr& state_msg){
		_current_state = *state_msg; 
	}
	
	void publishRange(const float filteredRange){
		
		// Populate sensor_msgs. 
		sensor_msgs::Range range; 
		range.header.stamp = ros::Time::now(); 
		range.header.frame_id = "camera_link"; 
		range.radiation_type = sensor_msgs::Range::INFRARED; 
		range.field_of_view = 2.0f*3.14f/3.0f;
		range.min_range = 0.2; 
		range.max_range = 2.0; 
		range.range = filteredRange;
		
		// Publish sensor_msgs
		_range_pub.publish(range); 
		
	}
	
	void imageCallback(const sensor_msgs::ImageConstPtr& depth_image){

		// Convert to OpenCV image
		cv_bridge::CvImagePtr depth_image_cv_ptr; 

		try{
			depth_image_cv_ptr = cv_bridge::toCvCopy(depth_image);  
		} catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception %s at %s, line %d", e.what(), __FILE__, __LINE__); 
			return; 
		}
		// Min and max values, and the locations 
		double minVal = 0, maxVal = 0;
		cv::Point minLoc, maxLoc; 
		
		cv::minMaxLoc(depth_image_cv_ptr->image, &minVal, &maxVal, &minLoc, &maxLoc, (depth_image_cv_ptr->image > 10)); 
		cv::circle(depth_image_cv_ptr->image, minLoc, 20, cv::Scalar(0) );
		_image_pub.publish(depth_image_cv_ptr->toImageMsg());
		
		// Convert to metres. 
		float curRange = minVal * 0.001; //minVal is in mm 
		
		if (isnan(_oldRange)) _oldRange = curRange; // Initialize if not set. 
		
		// Filter range. 
		float filteredRange = 0.3*curRange + 0.7*_oldRange; // Filter
		_oldRange = filteredRange; 
		
		// Publish. 
		// publishRange(filteredRange); 
		publishRange(curRange); // DEBUG. 

		// We are about to crash in a mission (AUTO.MISSION, AUTO.LOITER)
		if ( (filteredRange < 0.55) && ( (_current_state.mode == "AUTO.MISSION") || (_current_state.mode == "POSCTL" ) ) ) {
		  ROS_WARN("About to crash. Going into AUTO.LOITER"); 
		  mavros_msgs::SetMode loiter_set_mode; 
		  loiter_set_mode.request.custom_mode = "AUTO.LOITER"; 
	  	  // loiter_set_mode.request.base_mode = mavros_msgs::MAV_MODE::MAV_MODE_GUIDED_ARMED; 
		  _set_mode_cli.call(loiter_set_mode); 
		}
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


void find_min_distance(const cv::Mat& image, unsigned int* minVal, cv::Point* minLoc){

	size_t nRows = image.rows, nCols = image.cols;
	
	unsigned int curMinVal = 0xFFFF; 
	cv::Point curMinLoc = cv::Point(0, 0); 

	for (size_t i = 0; i < nRows; i++){
	  const unsigned int* imRow = image.ptr<unsigned int>(i); 
	  for (size_t j = 0; j < nCols; j++) {
	  	if ( (imRow[j] < curMinVal) && (imRow[j] > 10) ) {
		  curMinVal = imRow[j]; 
		  curMinLoc = cv::Point(i, j); 
		}		
	  }
	}
	*minVal = curMinVal; 
	*minLoc = curMinLoc; 
}
