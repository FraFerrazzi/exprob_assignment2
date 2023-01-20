#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class MoveCamera {

private:
    ros::NodeHandle nh_;
    // Initialize ROS clients, services, and publishers
    ros::Publisher move_cam_pub_;
    ros::Publisher move_base_pub_;
    ros::Subscriber marker_sub_;
    
    std_msgs::Float64 joint1_pos;
    std_msgs::Float64 joint2_pos;
    std_msgs::Int32 aruco_num;
    std_vector<int> ids_;
    std_msgs::Int32 marker_id;
    
public:
    MoveJoint() : it_(nh_) {
        move_cam_pub_ = nh_.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 1000);
        move_base_pub_ = nh_.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 1000);
        marker_sub_ = it_.subscribe("/marker_id", 1, &MoveCamera::arucoCallback, this); 
    }    
    
    void camerCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image_ = cv_ptr->image;
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(image_, dictionary_, corners_, ids_);
        if (ids_.size() > 0) {
            cv::aruco::drawDetectedMarkers(image_, corners_, ids_);
            cv::imshow("Aruco markers", image_);
            cv::waitKey(3);
            
            // Send the id of the detected markers
            for(std::size_t i = 0; i < ids_.size(); i++){
                room_srv_.request.id = ids_[i];
                if (marker_cli_.call(room_srv_)){
                    ROS_INFO("##################################\n\nDETECTED MARKER: %d\nRoom: %s\nCartesian coordinates: (%f,%f)\n\n", ids_[i], room_srv_.response.room.c_str(), room_srv_.response.x, room_srv_.response.y);
                    for(std::size_t j = 0; j < room_srv_.response.connections.size(); j++){
                        ROS_INFO("Connected to: %s\nDoor: %s\n\n##################################\n\n", room_srv_.response.connections.at(j).connected_to.c_str(), room_srv_.response.connections.at(j).through_door.c_str());
                    }
                }
            }
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joint");
    MoveJoint node;
    ros::spin();
    return 0;
}
