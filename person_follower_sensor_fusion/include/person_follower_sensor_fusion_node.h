


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
#include <chrono>

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

#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>

#include "opencv2/video/tracking.hpp"


#include "obstacles_detect_manager_.h"
#include "command_velocity_manager.h"
#include "params.h"


#ifndef PERSON_FOLLOWER_SENSOR_FUSION
#define PERSON_FOLLOWER_SENSOR_FUSION



using namespace std;
using namespace cv;
using namespace visualization_msgs;
using namespace std::chrono;

#define RESOULTION 50
#define MAP_W_METER 15
#define MAP_H_METER 15


class PersonFollowerSensorFusion
{

public:
    
    PersonFollowerSensorFusion();

    ~PersonFollowerSensorFusion();

private:

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void updateTimerCallback(const ros::TimerEvent&);

    void detectedObjectsCallback(const object_msgs::ObjectsInBoxes::Ptr &objects);
            
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info);

    void depthCallback(const sensor_msgs::ImageConstPtr &image);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void legsTargetsCallback(const geometry_msgs::PoseArrayConstPtr &legsMsg);

    bool extractDepthFromBboxObject( cv::Point2d pix,
            const Mat &depthImg, geometry_msgs::PointStamped& pose);

    int  calcTargetDegAngle(const cv::Point2d& targetPoint );
       

    geometry_msgs::PointStamped transformToByFrames(
        Point3d objectPoint3d, string base_Frame, string child_Frame) const ;

    bool findLargestDepthBlob(const cv::Rect& object_rect,
        const Mat& distanceTransformImg, cv::Point2d& depthPix);

    vector<FullTarget> createFullTargets(); 

    bool checkIfDepthIsNoiseByLaser(const geometry_msgs::PointStamped& objectPose);

    double distanceCalculate(cv::Point2d p1, cv::Point2d p2);

    bool pickBestTarget();

    bool getRobotPoseOdomFrame(geometry_msgs::Pose& robotPose);

    bool checkIfCanInitTaregt() const;

    double getRobotHeading(const geometry_msgs::Pose &pose) const;

     void publishTargetsMarkers();

    void publishDebugImg();

    string getStringState();


private:

    TrackingTarget trackingTarget_;

    ros::WallTime startSearching_ ;
    bool startSeacrhingFlag_ = false;
    
    /// params
    double algoRate_ = 20;
    double continuousSearchDuration_ = 5; //second
    double maxDistInitCameraTarget_  = 5.0; // meters
    double maxDistanceTracking_ = 10.0;
    double maxDistOnlyRotation_ = 0.5;
    double minSpeedPerSecond_ = 0.05;
    double maxSpeedPerSecond_ = 0.5;
    double angularScaleFactor_ = 0.5;


    //managers
    ObstaclesDetectManager obstaclesDetectManager_;
    CommandVelocityManager cmdVelManager_;

    //ROBOT POSE
    geometry_msgs::Pose lastRobotPose_;
    geometry_msgs::Pose currentRobotPose_;
    ros::Subscriber odomSubscriber_;

    //TWIST
    ros::Publisher twistCommandPublisher_;

    FOLLOWER_STATE followerState_;

    vector<FullTarget> currentFullTargets_;

    ros::Timer follweTimer_;
    string base_frame_id_ = "odom";

    // depth
    ros::NodeHandle node_;
    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    image_geometry::PinholeCameraModel pinholeCameraModel_;
    bool cameraInfoInited_ = false;
    string camera_depth_optical_frame_;
    cv::Mat currentDepthImg_;


    //rgb image
    image_transport::Publisher debugImgPublisher_;
    ros::Subscriber rgbSubscriber_;
    cv::Mat currentRgbImg_;
    bool initRgb_ = false;

    //laser
    ros::Subscriber laserScanSubscriber_;
    vector<cv::Point2d> currentScanPoints_;

    //targets
    string trackingTargetClassName_ = "person";
    vector<CameraTarget> currentCameraTargets_;
    vector<LegTarget> currentLegsTargets_;


    //objects
    ros::Subscriber objectSubscriber_;
    ros::Subscriber legsSubscriber_;   


    //tf
    tf::TransformListener tfListener_;

    //markers
    ros::Publisher cameraTargetsMarkerArr_;
    ros::Publisher legsTargetsMarkerArr_;

    //cmd-vel
    geometry_msgs::Twist lastCmdVel_;


    



};

#endif 