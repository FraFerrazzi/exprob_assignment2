/**
 * @file aruco_detection.cpp
 * @author Francesco Ferrazzi (s5262829@studenti.unige.it)
 * @brief Client node ode to get the informations from the aruco markers placed in the environment and send them to the FSM
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 * Subscribes to: <BR>
 *  /robot/camera1/image_raw to get images from camera
 * 
 * Services: <BR>
 *  /arm_info to make a request to the arm server
 *  /room_info to make a request to the marker server
 *  /world_init to make a request to the FSM
 * 
 * Description:
 * This node is a client node which makes request to the server nodes to:
 * 1) Make the arm of the robot rotate and tilt the camera to see all the aruco markers placed in the initial room
 * 2) Get the informations of the aruco markers from the markers server to get acquainted of the environment
 * 3) Send the informations to the Final State Machine node to build the map for the Robot
 * After all the markers have been detected and the informations have been sent, this node is shut down
 */
 
#include <ros/ros.h>
#include <algorithm> 
#include <std_msgs/Bool.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <sensor_msgs/image_encodings.h>

#include <exprob_assignment2/RoomInformation.h> 
#include <exprob_assignment2/RoomConnection.h> 
#include <exprob_assignment2/ArmInfo.h>
#include <exprob_assignment2/WorldInit.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>


/**
 * @brief Class for acquiring aruco markers and get and send the informations of the aruco markers
 * 
 */
class ArucoDetector {

private:
    ros::NodeHandle nh_;
    
    // Aruco stuff
    aruco::MarkerDetector detector_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::vector<aruco::Marker> detectedMarkers;
    std::vector<int> markerIDs;
    aruco::CameraParameters CamParam;
    // Constant values
    const double MARKER_SIZE = 0.05;
    const int MARKER_NUM = 7;
    
    // Initialize ROS clients, subscribers and messages
    ros::ServiceClient marker_cli_;
    exprob_assignment2::RoomInformation room_srv_;
    ros::ServiceClient camera_cli_;
    exprob_assignment2::ArmInfo arm_srv_;
    ros::ServiceClient world_cli_;
    exprob_assignment2::WorldInit world_srv_;
    exprob_assignment2::RoomConnection conn;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    
    cv::Mat image_;
    
public:
    /**
     * @brief Constructor of the Aruco Detector class, which initializes clients and subscribers
     * 
     */
    ArucoDetector() : it_(nh_) {
    	std::cout << "#######################################\nARUCO DETECTION node correctly launched\n#######################################\n\n"; 
    	std::cout << "Markers that needs to be detected: " << MARKER_NUM;
        img_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &ArucoDetector::imageCallback, this);
        camera_cli_ = nh_.serviceClient<exprob_assignment2::ArmInfo>("/arm_info");
        marker_cli_ = nh_.serviceClient<exprob_assignment2::RoomInformation>("/room_info");
        world_cli_ = nh_.serviceClient<exprob_assignment2::WorldInit>("/world_init");
        // Wait every node to be launched correctly before acquiring images
        ros::Duration(3).sleep();
        
        // Calibrate the camera
	CamParam = aruco::CameraParameters();
	
        // Wait for the services to be available
    	if (!marker_cli_.waitForExistence(ros::Duration(5))) {
        	ROS_ERROR("Service not found");
        	return;
    	}  
    	if (!camera_cli_.waitForExistence(ros::Duration(5))) {
        	ROS_ERROR("Service not found");
        	return;
    	} 
    	if (!world_cli_.waitForExistence(ros::Duration(5))) {
        	ROS_ERROR("Service not found");
        	return;
    	} 
    }    
    
    /**
     * @brief Function called every time an image is pulished on the '/robot/camera1/image_raw' topic.
     * 
     * @param msg is the variable which contains the message which was published
     * 
     * This function gets the messages and process them to retrieve the IDs of the aruco markers contained in the field of
     * view of the camera placed on the robot. Once the marker is detected, if it is the first time that has been
     * seen, the ID is saved and sent to the aruco server to get the informations belonging to the specific marker.
     * Once the informations have received, the same infos are sent to the final state machine to be update the ontology.
     * If the ID was already detected, nothing is done.
     * Once all the markers are detected, the node is shut down.
     */
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
        // If at least one marker is detected
        if (detectedMarkers.size() > 0) {
            cv::imshow("Aruco markers", image_);
            cv::waitKey(3);
            for(std::size_t i = 0; i < detectedMarkers.size(); i++){
                // If the id was not detected yet, add it to the array and do things
                if (std::find(markerIDs.begin(), markerIDs.end(), detectedMarkers.at(i).id) == markerIDs.end()) {
    	            markerIDs.push_back(detectedMarkers.at(i).id);
    	            // Send the id of the detected markers
                    room_srv_.request.id = detectedMarkers.at(i).id;
                    // Get the informations of the detected marker
                    if (marker_cli_.call(room_srv_)){
                        ROS_INFO("\n\n#######################################\nDETECTED MARKER: %d\nRoom: %s\nCartesian coordinates: (%f,%f)\n", detectedMarkers.at(i).id, room_srv_.response.room.c_str(), room_srv_.response.x, room_srv_.response.y);
                        // Generate the request to send the retrieved informations to the state machine node
                        world_srv_.request.room = room_srv_.response.room;
                        world_srv_.request.x = room_srv_.response.x;
                        world_srv_.request.y = room_srv_.response.y;
                        for(std::size_t j = 0; j < room_srv_.response.connections.size(); j++){
                            ROS_INFO("Connected to: %s, with door: %s", room_srv_.response.connections.at(j).connected_to.c_str(), room_srv_.response.connections.at(j).through_door.c_str()); 
                            conn.connected_to = room_srv_.response.connections.at(j).connected_to;
                            conn.through_door = room_srv_.response.connections.at(j).through_door;
                            world_srv_.request.connections.push_back(conn);
                        }
                        // Call the service
                        if (world_cli_.call(world_srv_)){
                        	ROS_INFO("\nInfos have been sent to the FSM node!\n#######################################");
                        }
                    }
                }
	        } 
        }
        // When all the markers will be detected send a message to the move_cam node and
        // se the detection_not_done to false to end this program
        if (markerIDs.size() == MARKER_NUM){
            arm_srv_.request.done = 1;
            ROS_INFO("\n\nALL MARKERS HAVE BEEN DETECTED\n");
            // Call the service to stop the camera
            if (camera_cli_.call(arm_srv_)) {
                if (arm_srv_.response.success) {
                    ROS_INFO("Service call succeeded");
                } else {
                    ROS_ERROR("Service call failed");
                }
            } else {
                ROS_ERROR("Failed to call service.");
                return;
            }
            // ROS shutdown
            ros::shutdown();
        }
    }

};

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return 0
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_detection");
    ArucoDetector node;
    ros::Rate loop_rate(50);
    while(ros::ok()) {
    	ros::spinOnce();
    	loop_rate.sleep();
    }
    return 0;
}

