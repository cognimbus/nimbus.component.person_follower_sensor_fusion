


#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <object_msgs/ObjectInBox.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <boost/algorithm/string.hpp>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <object_msgs/ObjectsInBoxes.h>

#include <cv_bridge/cv_bridge.h>

#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>



#ifndef PARAMS_H
#define PARAMS_H


using namespace std;
using namespace cv;

enum FOLLOWER_STATE
{
    IDLE,
    INIT_TARGET,
    DETECT_PERSONS,
    RECOGNIZE_TARGET,
    CHECK_TARGET_COLLISION,
    SEARCHING_360_TARGET,
    TRACKING,
    SEARCHING 
    
};

enum TRACKING_MODE 
{   
    NONE,
    CAME_FROM_INIT,
    CAME_FROM_TRACKING
};




struct CameraTarget {

    geometry_msgs::PointStamped position_ ;
    cv::Rect rgb_bounding_box_ ;
    cv::Point2d rgbPix_ ;
    double degAngle_ = -1;
    double distance_ = 0;
};

struct LegTarget {

    geometry_msgs::PointStamped position_ ;
    double degAngle_ = -1;
    double distance_ = 0;
};

struct FullTarget {

    CameraTarget cameraTarget_;
    LegTarget legTarget_;

    geometry_msgs::PointStamped mergedTargetPosition_;
    double mergeTargetAngle_;

    bool hasCamera_ = false;
    bool hasLeg_ = false;

    
};

struct TrackingTarget {

    double x_pos_pred;
    double y_pos_pred;

    FullTarget fullTarget_;

    geometry_msgs::PointStamped globalPosition_;

    double globalDistance_;

    double globalDegAngle_;

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

    cv::Mat state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    void initKalmanParams() {
        
       cv::setIdentity(kf.transitionMatrix);
       kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
       kf.measurementMatrix.at<float>(0) = 1.0f;
       kf.measurementMatrix.at<float>(7) = 1.0f;
       kf.measurementMatrix.at<float>(16) = 1.0f;
       kf.measurementMatrix.at<float>(23) = 1.0f;

       kf.processNoiseCov.at<float>(0) = 1e-2;
       kf.processNoiseCov.at<float>(7) = 1e-2;
       kf.processNoiseCov.at<float>(14) = 5.0f;
       kf.processNoiseCov.at<float>(21) = 5.0f;
       kf.processNoiseCov.at<float>(28) = 1e-2;
       kf.processNoiseCov.at<float>(35) = 1e-2;

       cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
 
    }

    void updateKalmanMeas()  {

        meas.at<float>(0) = globalPosition_.point.x;
        meas.at<float>(1) = globalPosition_.point.y;
        meas.at<float>(2) = 0.2;
        meas.at<float>(3) = 0.2;
    }
    void updateKalmanParams() {

        updateKalmanMeas();    

        kf.errorCovPre.at<float>(0) = 1; // px
        kf.errorCovPre.at<float>(7) = 1; // px
        kf.errorCovPre.at<float>(14) = 1;
        kf.errorCovPre.at<float>(21) = 1;
        kf.errorCovPre.at<float>(28) = 1; // px
        kf.errorCovPre.at<float>(35) = 1; // px

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(1) = meas.at<float>(1);
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;
        state.at<float>(4) = meas.at<float>(2);
        state.at<float>(5) = meas.at<float>(3);
        // <<<< Initialization

        kf.statePost = state;
    }
    

    void updateGlobalPositionAndAngle() {

        if( fullTarget_.hasCamera_  == true && fullTarget_.hasLeg_ == true){

            // merged         
            globalPosition_ = fullTarget_.mergedTargetPosition_;

            globalDegAngle_ = fullTarget_.mergeTargetAngle_;

            globalDistance_ = fullTarget_.cameraTarget_.distance_;

        } // only camera
        else  if( fullTarget_.hasCamera_  == true && fullTarget_.hasLeg_ == false) {            

            globalPosition_ = fullTarget_.cameraTarget_.position_;     
            globalDegAngle_ =  fullTarget_.cameraTarget_.degAngle_;

            globalDistance_ = fullTarget_.cameraTarget_.distance_;     
        }         
        else { // only leg
         
            globalPosition_ = fullTarget_.legTarget_.position_; 
            globalDegAngle_ =  fullTarget_.legTarget_.degAngle_;  
            globalDistance_ = fullTarget_.legTarget_.distance_;                 
        }
    }
   
};


#endif 