#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <exprob_assignment2/ArmInfo.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// Global variable
bool detection_done = false;

class MoveJoint {

private:
    ros::NodeHandle nh_;
    // Initialize ROS clients, services, and publishers
    ros::Publisher move_cam_pub;
    ros::Publisher move_base_pub;
    ros::ServiceServer motion_srv;
    // Variables initialization
    std_msgs::Float64 joint1_pos;
    std_msgs::Float64 joint2_pos;
    bool move_arm = true;
    bool cam_up = true;
    double current_base_pos = 0; // Variable to store the current base joint position
    double current_cam_pos = 0; // Variable to store the current camera joint position
    double full_base_cycle = 6.2;
    double cam_lim = 0.2;
    
public:
    MoveJoint() {
        move_cam_pub = nh_.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 10);
        move_base_pub = nh_.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
        motion_srv = nh_.advertiseService("/arm_info", &MoveJoint::motionCallback, this); 
        ROS_INFO("Move the arm until markers have been detected!\n");
        while(move_arm){
            rotate_camera_joint(move_cam_pub);
            rotate_base_joint(move_base_pub);
            cam_up = !cam_up;
            ros::spinOnce();
        }
    } 
    
    void rotate_camera_joint(ros::Publisher move_cam_pub) {
        // Set the rate of the loop to run at 10 Hz
        ros::Rate loop_rate(10);
        // If the camera should go up
        if(cam_up){
            // Loop until the current camera position is equal to 30 degrees
	    while (current_cam_pos < cam_lim)
	    {
                // Create a message to store the joint position
		joint2_pos.data = current_cam_pos;
	        // Publish the message to the robot/joint2_position_controller/command topic
	        move_cam_pub.publish(joint2_pos);
		// Increment the current position by 0.5 degrees
		current_cam_pos += 0.1;
	        // Sleep for the time remaining until the next loop iteration
	        loop_rate.sleep();
	    }
        }
        else {
            // Loop until the current camera position is equal to -30 degrees
            while (current_cam_pos > -cam_lim)
	    {
                // Create a message to store the joint position
		joint2_pos.data = current_cam_pos;
	        // Publish the message to the robot/joint2_position_controller/command topic
	        move_cam_pub.publish(joint2_pos);
		// Decrement the current position by 0.5 degrees
		current_cam_pos -= 0.1;
	        // Sleep for the time remaining until the next loop iteration
	        loop_rate.sleep();
	    }
        }
    }  
    
    void rotate_base_joint(ros::Publisher move_base_pub) {
        // Set the rate of the loop to run at 10 Hz
        ros::Rate loop_rate(10);
        // Loop until the current base position is equal to home position
        while (current_base_pos < full_base_cycle)
        {
            // Create a message to store the joint position
            joint1_pos.data = current_base_pos;
            // Publish the message to the robot/joint1_position_controller/command topic
            move_base_pub.publish(joint1_pos);
            // Increment the current position by 1 degree
            current_base_pos += 0.1;
            // Sleep for the time remaining until the next loop iteration
            loop_rate.sleep();
        }
        current_base_pos = 0;        
    }  
    
    void move_to_surv_pos(ros::Publisher move_base_pub, ros::Publisher move_cam_pub) {
        // Set the rate of the loop to run at 20 Hz
        ros::Rate loop_rate(20);
        // Loop until the current position is equal to 360 degrees
        while (current_base_pos > 0)
        {
            joint1_pos.data = current_base_pos;
            move_base_pub.publish(joint1_pos);
            current_base_pos -= 0.1;
            loop_rate.sleep();
        }
        current_base_pos = 0;
        joint1_pos.data = current_base_pos;
        move_base_pub.publish(joint1_pos);
        // Loop until the current camera position is equal to home position
        while (current_base_pos > 0) {
	    joint2_pos.data = current_cam_pos;
	    move_cam_pub.publish(joint2_pos);
	    current_cam_pos -= 0.1;
	    loop_rate.sleep();
        }
        current_cam_pos = 0;
        joint2_pos.data = current_cam_pos;
        move_cam_pub.publish(joint2_pos);
    }  
    
    bool motionCallback(exprob_assignment2::ArmInfo::Request &req, exprob_assignment2::ArmInfo::Response &res) {
        move_arm = false;
        req.done;
        ROS_INFO("ALL THE MARKERS HAVE BEEN DETECTED\n\nArm is going to home position!\n");
        move_to_surv_pos(move_base_pub, move_cam_pub);
        res.success = 1;
        ROS_INFO("ROS Shutdown\n");
        // Wait few seconds to let the user see the output
        detection_done = true;
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joint");
    MoveJoint node;
    ros::Rate loop_rate(50);
    while(!detection_done){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
