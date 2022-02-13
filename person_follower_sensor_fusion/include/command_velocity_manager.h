


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


#include "params.h"


#ifndef COMMAND_VELOCITY_MANAGER
#define COMMAND_VELOCITY_MANAGER


using namespace std;
using namespace cv;

class CommandVelocityManager
{

public:
    
    CommandVelocityManager(){
        
    }


    ~CommandVelocityManager(){}

public:


    void initParams(double  maxDistOnlyRotation, double minSpeedPerSecond, 
        double maxSpeedPerSecond, double angularScaleFactor, double maxDistanceTracking) {

        maxDistanceForOnlyRotation_ = maxDistOnlyRotation;

        angularScaleFactor_ = angularScaleFactor;

        minSpeed_ = minSpeedPerSecond;

        maxSpeed_ = maxSpeedPerSecond;

        maxDistanceForLinear_ = maxDistanceTracking;
    }
    double calcAngularVelocity(double degAngle) { 


        double scaleFactor = angularScaleFactor_;

        double angular_z = 0;
        if ( degAngle > 0 && degAngle <= 180) {
            // turn left
            angular_z =  -1 * angles::from_degrees(degAngle) * scaleFactor ;
        } else  {
            // turn right
            angular_z =  1 * angles::from_degrees(degAngle ) * 0.25;
        }

        return angular_z;

    }   

    double calcLinearVelocity(double distance, double degAngle) { 

        // only rotation
        if( distance < maxDistanceForOnlyRotation_){

            return 0;            
        }       

         // only rotation, the target behimd the robot
        if (degAngle >= 135 && degAngle <= 225) {
            return 0;
        }

        return normalizedSpeed(distance,  minSpeed_,  maxSpeed_);

    }   

    double normalizedSpeed(double distance, double minSpeed, double maxSpeed) {

        double old_value = distance;
        double old_min = 0.25;
        double old_max = maxDistanceForLinear_;
        double new_min = minSpeed;
        double new_max = maxSpeed;

        double normalizedSpeeddSpeed = ( ( old_value - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min ;
            
       
        
        return normalizedSpeeddSpeed;
    }

    geometry_msgs::Twist createCmdVel(const TrackingTarget trackingTarget) {      
        

        geometry_msgs::Twist command;        

        // merged target
        if (trackingTarget.fullTarget_.hasCamera_ && trackingTarget.fullTarget_.hasLeg_) {

            command.linear.x = calcLinearVelocity( trackingTarget.fullTarget_.cameraTarget_.distance_, 
                trackingTarget.fullTarget_.cameraTarget_.degAngle_);
                
            command.angular.z = calcAngularVelocity(trackingTarget.fullTarget_.cameraTarget_.degAngle_);
            
            return command;

        } //only camera 
        else if ( trackingTarget.fullTarget_.hasCamera_== true && trackingTarget.fullTarget_.hasLeg_ == false) {

            command.linear.x = calcLinearVelocity( trackingTarget.fullTarget_.cameraTarget_.distance_,
                trackingTarget.fullTarget_.cameraTarget_.degAngle_);
               
            command.angular.z = calcAngularVelocity(trackingTarget.fullTarget_.cameraTarget_.degAngle_);
            
            return command;

        } //only leg
        else if ( trackingTarget.fullTarget_.hasCamera_ == false && trackingTarget.fullTarget_.hasLeg_ == true) {

            command.linear.x = calcLinearVelocity( trackingTarget.fullTarget_.legTarget_.distance_, 
                trackingTarget.fullTarget_.legTarget_.degAngle_);
               
            command.angular.z = calcAngularVelocity(trackingTarget.fullTarget_.legTarget_.degAngle_);
                
            return command;

        } else {

            cerr<<" eeeeeeeeeeeeeeerrrrrrrrrrror "<<endl;
        }

        return command;
      
   }

    geometry_msgs::Twist createZeroCmdVel() { 

        geometry_msgs::Twist command;
        command.linear.x = 0;
        command.angular.z = 0;
        
        return command;      

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

private:

    double maxDistanceForOnlyRotation_;

    double angularScaleFactor_;

    double minSpeed_;

    double maxSpeed_;

    double maxDistanceForLinear_ = 5; //meters

};

#endif 