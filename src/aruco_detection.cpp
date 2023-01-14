#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector {

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::vector<int> ids_;
    std::vector<std::vector<cv::Point2f> > corners_;
    aruco::MarkerDetector detector_;
    
    ros::ServiceClient marker_cli_;
    
    cv::Mat image_;
    
public:
    ArucoDetector() : it_(nh_) {
        img_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &ArucoDetector::imageCallback, this);
        marker_cli = nh.serviceClient<exprob_assignment2::RoomInformation>("/room_info")
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
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
        cv::aruco::detectMarkers(image_, dictionary_, corners_, ids_);
        if (ids_.size() > 0) {
            cv::aruco::drawDetectedMarkers(image_, corners_, ids_);
            cv::imshow("Aruco markers", image_);
            cv::waitKey(3);
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detection");
    ArucoDetector aruco_detector;
    ros::spin();
    return 0;
}

