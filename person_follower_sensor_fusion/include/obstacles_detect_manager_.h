


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

#ifndef OBSTACLES_DETECT_MANAGER
#define OBSTACLES_DETECT_MANAGER


using namespace std;
using namespace cv;

class ObstaclesDetectManager
{

public:
    
    ObstaclesDetectManager(){}

    ~ObstaclesDetectManager(){}

public:

   bool detectObstacle(const vector<cv::Point2d>& currentScanPoints,
     const geometry_msgs::Pose& currentRobotPose, double maxDistance = 0.25) {


        for (int j = 0; j < currentScanPoints.size(); j++) {           


            double dist = distanceCalculate(
                cv::Point2d(currentScanPoints[j].x, currentScanPoints[j].y), 
                cv::Point2d(currentRobotPose.position.x, currentRobotPose.position.y));
                
             
            if(  dist < maxDistance && dist > 0.2) {
                return true; 
            }
        }
        
        return false;
        

   }

private:    

    double distanceCalculate(cv::Point2d p1, cv::Point2d p2) {
        double x = p1.x - p2.x; //calculating number to square in next step
        double y = p1.y - p2.y;
        double dist;

        dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
        dist = sqrt(dist);

        return dist;
    }

};

#endif 