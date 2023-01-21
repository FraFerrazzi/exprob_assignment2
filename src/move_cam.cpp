#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

// Global variable
bool detection_not_done = true;

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
    bool move_cam = true;
    
public:
    MoveJoint() : it_(nh_) {
        move_cam_pub_ = nh_.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 1000);
        move_base_pub_ = nh_.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 1000);
        marker_sub_ = nh_.subscribe("bool_topic", 1, boolCallback, this); 
        while(move_cam){
        
        }
    }    
    
    void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
        move_cam = msg->data;
        ROS_INFO("ALL THE MARKERS HAVE BEEN DETECTED\n\nStop the arm motion and shutdown!\n");
        // Wait few seconds to let the user see the output
        ros::Duration(3.0).sleep();
        detection_not_done = false;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joint");
    MoveJoint node;
    while(detection_not_done){
        ros::spin();
    }
    return 0;
}
