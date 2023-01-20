#include <ros/ros.h>
#include <algorithm> 
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <sensor_msgs/image_encodings.h>

#include <exprob_assignment2/RoomInformation.h> 
#include <exprob_assignment2/RoomConnection.h> 

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

// Global variable
bool detection_not_done = true;

class ArucoDetector {

private:
    ros::NodeHandle nh_;
    
    aruco::MarkerDetector detector_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::vector<aruco::Marker> detectedMarkers;
    std::vector<int> markerIDs;
    aruco::CameraParameters CamParam;
    double marker_size = 0.05;
    int markers_num = 7;
    
    // Initialize ROS clients, services, publishers and subscribers
    ros::ServiceClient marker_cli_;
    exprob_assignment2::RoomInformation room_srv_;
    ros::Publisher marker_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    
    cv::Mat image_;
    
public:
    ArucoDetector() : it_(nh_) {
    	std::cout << "#######################################\nARUCO DETECTION node correctly launched\n#######################################\n\n"; 
    	std::cout << "Markers that needs to be detected: 7";
        img_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &ArucoDetector::imageCallback, this);
        //marker_pub_ = nh_.advertise<std_msgs::Int32>("/marker_id");
        marker_cli_ = nh_.serviceClient<exprob_assignment2::RoomInformation>("/room_info");
        
        // Calibrate the camera
	CamParam = aruco::CameraParameters();

        // Wait for the service to be available
    	if (!marker_cli_.waitForExistence(ros::Duration(5))) {
        	ROS_ERROR("Service not found");
        	return;
    	}  
    }    
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image_ = cv_ptr->image;
        // Clear already detected markers
        detectedMarkers.clear();
        // Detect the marker
        detector_.detect(image_, detectedMarkers, CamParam);
        if (detectedMarkers.size() > 0) {
            cv::imshow("Aruco markers", image_);
            cv::waitKey(3);
            for(std::size_t i = 0; i < detectedMarkers.size(); i++){
                // If the ids was not detected yet, add it to the array and do things
                if (std::find(markerIDs.begin(), markerIDs.end(), detectedMarkers.at(i).id) == markerIDs.end()) {
    	            markerIDs.push_back(detectedMarkers.at(i).id);
    	            // Send the id of the detected markers
                    room_srv_.request.id = detectedMarkers.at(i).id;
                    if (marker_cli_.call(room_srv_)){
                        ROS_INFO("\n\n##################################\nDETECTED MARKER: %d\nRoom: %s\nCartesian coordinates: (%f,%f)\n", detectedMarkers.at(i).id, room_srv_.response.room.c_str(), room_srv_.response.x, room_srv_.response.y);
                        for(std::size_t j = 0; j < room_srv_.response.connections.size(); j++){
                            ROS_INFO("Connected to: %s, with door: %s", room_srv_.response.connections.at(j).connected_to.c_str(), room_srv_.response.connections.at(j).through_door.c_str());
                        }
                    }
                }
	    } 
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_detection");
    ArucoDetector node;
    while(detection_not_done) {
    	ros::spin();
    }
    return 0;
}

