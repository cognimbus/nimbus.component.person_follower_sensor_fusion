
#include "../include/person_follower_sensor_fusion_node.h"

//rosbag play persons.bag --topics /tf /color_image_raw /openvino_toolkit/detected_objects /camera/depth/image_rect /camera/depth/image_rect/camera_info /scan -->

PersonFollowerSensorFusion::~PersonFollowerSensorFusion() {}

PersonFollowerSensorFusion::PersonFollowerSensorFusion()
{

    ros::NodeHandle node_;
    ros::NodeHandle nodePrivate("~");

    //ros pararms
    nodePrivate.param("enable_scan_targets", enalbeScanTargets_, false);
    nodePrivate.param("algo_rate", algoRate_, 30.0);
    nodePrivate.param("target", trackingTargetClassName_,  string("person")); 
    nodePrivate.param("max_dist_init_camera_target", maxDistInitCameraTarget_, 5.0); 
    nodePrivate.param("continuous_search_duration", continuousSearchDuration_, 5.0); 
    nodePrivate.param("max_distance_tracking", maxDistanceTracking_, 5.0);
    nodePrivate.param("max_distance_only_rotation", maxDistOnlyRotation_, 1.0); 
    nodePrivate.param("min_speed_per_second", minSpeedPerSecond_, 0.05); 
    nodePrivate.param("max_speed_per_second", maxSpeedPerSecond_, 0.5); 
    nodePrivate.param("angular_scale_factor", angularScaleFactor_, 0.8); 

    //timer
    startStopnSubscriber_ = node_.subscribe("/start_stop", 1,
        &PersonFollowerSensorFusion::startStopCallback, this);   

    
    follweTimer_ = node_.createTimer(ros::Rate(algoRate_), 
            &PersonFollowerSensorFusion::updateTimerCallback, this);  

    // subscribers
    rgbSubscriber_ = node_.subscribe("/color_image_raw", 1,
            &PersonFollowerSensorFusion::imageCallback, this);

    objectSubscriber_ = node_.subscribe("/openvino_toolkit/detected_objects", 1,
            &PersonFollowerSensorFusion::detectedObjectsCallback, this);

    odomSubscriber_ = node_.subscribe("/odom", 1,
            &PersonFollowerSensorFusion::odomCallback, this);        

    image_depth_sub.subscribe(node_, "/camera/depth/image_rect", 1);
    image_depth_sub.registerCallback(&PersonFollowerSensorFusion::depthCallback, this);

    info_sub.subscribe(node_, "/camera/depth/image_rect/camera_info", 1);
    info_sub.registerCallback(&PersonFollowerSensorFusion::cameraInfoCallback, this);   

    laserScanSubscriber_ = node_.subscribe("/scan", 1,
        &PersonFollowerSensorFusion::scanCallback, this);

    legsSubscriber_ = node_.subscribe("/legs_targets", 1,
        &PersonFollowerSensorFusion::legsTargetsCallback, this);   

    // publishers
    twistCommandPublisher_ 
        = node_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, false);          
  

    image_transport::ImageTransport it(node_);
    debugImgPublisher_ = it.advertise("/debug_img", 1);
    node_.param("/debug_img/compressed/jpeg_quality", 20);   


    cameraInfoInited_ = false; 
    initRgb_ = false;
    followerState_ = IDLE;
    mode_ = false;


    trackingTarget_.initKalmanParams(); 


    cmdVelManager_.initParams(maxDistOnlyRotation_, minSpeedPerSecond_,
         maxSpeedPerSecond_, angularScaleFactor_, maxDistanceTracking_);

    //markers
    cameraTargetsMarkerArr_ = node_.advertise<visualization_msgs::MarkerArray>("/camera_targets_markers", 10);
    legsTargetsMarkerArr_ = node_.advertise<visualization_msgs::MarkerArray>("/legs_targets_markers", 10);

  

} 

void PersonFollowerSensorFusion::startStopCallback(const std_msgs::Bool::ConstPtr& msg){

    if( msg->data == true){

        mode_ = true;
    
    } else {
        mode_ = false;
    }
}
void PersonFollowerSensorFusion::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    currentRobotPose_ = msg->pose.pose;

}

void PersonFollowerSensorFusion::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{   

    currentScanPoints_.clear();

    for (double i = 0; i < scan->ranges.size(); i++)
    {

        if (isinf(scan->ranges[i]) == false )
        {

            double ray_angle = scan->angle_min + (i * scan->angle_increment);

            cv::Point2d rayPoint((scan->ranges[i] * cos(ray_angle)),
                                    (scan->ranges[i] * sin(ray_angle)));

            // transform to bas-link
            cv::Point3d p = cv::Point3d(rayPoint.x, rayPoint.y, 0);

            auto transformedRayPoint = transformToByFrames(p, base_frame_id_ , scan->header.frame_id );                        

            currentScanPoints_.push_back(cv::Point2d(transformedRayPoint.point.x, transformedRayPoint.point.y));
        }
    }
    
}

void PersonFollowerSensorFusion::legsTargetsCallback(const geometry_msgs::PoseArrayConstPtr &legsMsg){


    currentLegsTargets_.clear();

    if (cameraInfoInited_) {

        for(int i = 0; i < legsMsg->poses.size(); i++) {

           
            cv::Point3d p = cv::Point3d(legsMsg->poses[i].position.x , legsMsg->poses[i].position.y,
                legsMsg->poses[i].position.z);

            auto odomFrameTarget =  transformToByFrames(p, base_frame_id_ , legsMsg->header.frame_id );  

            LegTarget legTarget;
            
            geometry_msgs::PointStamped legsTarget;
            legTarget.position_.header.frame_id = legsMsg->header.frame_id;
            legTarget.position_.header.stamp = ros::Time(0);
            legTarget.position_.point.x = odomFrameTarget.point.x;
            legTarget.position_.point.y = odomFrameTarget.point.y;
            legTarget.position_.point.z = odomFrameTarget.point.z;

            legTarget.degAngle_ = calcTargetDegAngle(cv::Point2d(legTarget.position_.point.x, legTarget.position_.point.y));
            legTarget.distance_ =  distanceCalculate(cv::Point2d(legTarget.position_.point.x, legTarget.position_.point.y),
                     cv::Point2d(currentRobotPose_.position.x, currentRobotPose_.position.y));

            
            currentLegsTargets_.push_back(legTarget);   
        }
    }


}


int  PersonFollowerSensorFusion::calcTargetDegAngle(const cv::Point2d& targetPoint ) {


    // clockwise 
    geometry_msgs::PointStamped targetPointBaseLink = transformToByFrames(cv::Point3d(targetPoint.x, targetPoint.y, 0), "base_link", "odom");

    cv::Point2d robotRef(0,0);
    cv::Point2d transformedTargetPoint(targetPointBaseLink.point.x, targetPointBaseLink.point.y);
    
    int angle = (int)(angles::to_degrees(atan2(transformedTargetPoint.y - 0, transformedTargetPoint.x - 0))) ;
    if (angle < 0 ){
        angle *= -1;
    } else {
        angle = 360 - angle;
    }
   
    if( angle > 360){
        angle  = angle % -360 ;
    }

    return angle;
    

}
void PersonFollowerSensorFusion::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info) {

    if ( cameraInfoInited_ == false)
    {
        if ( pinholeCameraModel_.fromCameraInfo(*cam_info) && initRgb_)
        {
            cameraInfoInited_ = true;
        }
    }
}

void PersonFollowerSensorFusion::depthCallback(const sensor_msgs::ImageConstPtr &image) {

    if (cameraInfoInited_ && initRgb_) {

        camera_depth_optical_frame_ = image->header.frame_id;

        cv_bridge::CvImagePtr cvBridgePtr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

        currentDepthImg_ = cvBridgePtr->image;      
    }
}

bool PersonFollowerSensorFusion::extractDepthFromBboxObject( cv::Point2d pix,
            const Mat &depthImg, geometry_msgs::PointStamped& pose) {

    float cx = pinholeCameraModel_.cx();
    float cy = pinholeCameraModel_.cy();

    float fx = pinholeCameraModel_.fx();
    float fy = pinholeCameraModel_.fy();

    float d = depthImg.at<float>(pix.y, pix.x) / 1000; /// IN METERS
    

    if( isinf(d) || d == 0){
        return false;
    }
    cv::Point3d p = cv::Point3d(((pix.x - cx) * d / fx), ((pix.y - cy) * d / fy), d);

    auto objectPose = transformToByFrames(p, base_frame_id_ ,camera_depth_optical_frame_ );

    pose =  objectPose;

    return true;
}

geometry_msgs::PointStamped PersonFollowerSensorFusion::transformToByFrames(
        Point3d objectPoint3d, string base_Frame, string child_Frame) const {
      
       
    geometry_msgs::PointStamped pointStampedIn;
    geometry_msgs::PointStamped pointStampedOut;

    pointStampedIn.header.frame_id = child_Frame;
    pointStampedIn.header.stamp = ros::Time(0);
    pointStampedIn.point.x = objectPoint3d.x;
    pointStampedIn.point.y = objectPoint3d.y;
    pointStampedIn.point.z = objectPoint3d.z;
    
    if (tfListener_.waitForTransform(base_Frame, child_Frame, 
            ros::Time(0), ros::Duration(0.1))) {

        tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);
        return pointStampedOut;

    } else {
        cerr<<"Failed to find transform between "<<base_Frame<<" and "<<child_Frame<<endl;
        return pointStampedOut;
    }

}

void PersonFollowerSensorFusion::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        currentRgbImg_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        initRgb_ = true;
       
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void PersonFollowerSensorFusion::exectueSensorFusionStateMachine(){


}

void PersonFollowerSensorFusion::sendGoalToTarget(){



}
void PersonFollowerSensorFusion::exectueCameraStateMachine(){

   

    switch (followerState_)
    {
        
        case IDLE: {  

            cerr<<" IDLE "<<endl;

            // nimbus stream pub person-follower-for-paul start_stop '{"data": 'true'}'
            if( mode_ == true){

                followerState_ = INIT_TARGET;

                break;             
            } else {
                
                auto cmdVel = cmdVelManager_.createZeroCmdVel();
                twistCommandPublisher_.publish(cmdVel);

                break;  
            }
        }
        case INIT_TARGET: {  

            cerr<<" INIT_TARGET "<<endl;

            if( checkIfCanInitTaregt()){

                trackingTarget_.fullTarget_ = currentFullTargets_[0];
                trackingTarget_.updateGlobalPositionAndAngle();               

                trackingState_ = CAME_FROM_INIT;
                followerState_ = DETECT_PERSONS;
                break;
                

            } else {

                followerState_ = IDLE;

                auto cmdVel = cmdVelManager_.createZeroCmdVel();
                twistCommandPublisher_.publish(cmdVel);

                break;
            }
        }
        case DETECT_PERSONS: {  

            cerr<<" DETECT_PERSONS "<<endl;

            if( currentFullTargets_.size() > 0){
                
                followerState_ = RECOGNIZE_TARGET;                
            } 
            else {

                if( trackingState_ == CAME_FROM_INIT ){

                    followerState_ = IDLE;
                    break;
                }
                if( trackingState_ == CAME_FROM_TRACKING ){

                    followerState_ = SEARCHING_360_TARGET;
                    break;
                }
            }
        }
        case RECOGNIZE_TARGET: {  

            cerr<<" RECOGNIZE_TARGET "<<endl;

            trackingState_ = CAME_FROM_TRACKING;

            recognizeTarget();

            bool res = checkTargetCollision();

            if( !res) {
                // no collsions
                auto cmdVel = cmdVelManager_.createCmdVel(trackingTarget_);            
                twistCommandPublisher_.publish(cmdVel);

                followerState_ = DETECT_PERSONS;
                break;
            
            } else {

                // there collision ,send goal to the target
                sendGoalToTarget();
                
                followerState_ = DETECT_PERSONS;
                break;
            }
        }
        case SEARCHING_360_TARGET: { 

            cerr<<" RECOGNIZE_TARGET "<<endl;

        }
    }

}

void PersonFollowerSensorFusion::recognizeTarget(){


}

bool PersonFollowerSensorFusion::checkTargetCollision(){


    return false;
}
void PersonFollowerSensorFusion::updateTimerCallback(const ros::TimerEvent&) {


    if ( !mode_ ){

        cerr<<" mode is "<<mode_<<endl;
        followerState_ = IDLE;
        return;
    }   

    if( !cameraInfoInited_ ||  !initRgb_ ){

        followerState_ = IDLE;
    } 

    if( !enalbeScanTargets_ ){

        exectueCameraStateMachine();

    } else {

        exectueSensorFusionStateMachine();
    }
    // if ( getRobotPoseOdomFrame(currentRobotPose_)) {

    //     currentFullTargets_ = createFullTargets();

    //     // obstacle detection - ROBOT WILL STOP
    //     if (obstaclesDetectManager_.detectObstacle(currentScanPoints_, currentRobotPose_)) {

    //         cerr<<" detectObstacle, return "<<endl;
    //         auto cmdVel = cmdVelManager_.createZeroCmdVel();

    //         twistCommandPublisher_.publish(cmdVel);

    //         followerState_ = IDLE;

    //         publishDebugImg();

    //         return;

    //     }


    //     switch (followerState_)
    //     {

    //         case IDLE: {  

    //             cerr<<"IDLE"<<endl;            
    //             if( checkIfCanInitTaregt()){


    //                 trackingTarget_.fullTarget_ = currentFullTargets_[0];
    //                 trackingTarget_.updateGlobalPositionAndAngle();

    //                 trackingTarget_.updateKalmanParams();
                    

    //                 auto cmdVel = cmdVelManager_.createCmdVel(trackingTarget_);
                
    //                 twistCommandPublisher_.publish(cmdVel);

    //                 followerState_ = TRACKING;

    //                 break;

    //             } else {

    //                 followerState_ = IDLE;

    //                 auto cmdVel = cmdVelManager_.createZeroCmdVel();

    //                 twistCommandPublisher_.publish(cmdVel);

    //                 break;
    //             }

    //         }
    //         case TRACKING: {
                
    //             cerr<<"TRACKING"<<endl;    

    //             // FullTarget target;
    //             if( pickBestTarget()) {
                    
                    

    //                 trackingTarget_.updateKalmanMeas();

    //                 trackingTarget_.kf.correct(trackingTarget_.meas);


    //                 auto cmdVel = cmdVelManager_.createCmdVel(trackingTarget_);
    //                 twistCommandPublisher_.publish(cmdVel);

    //                 lastCmdVel_ = cmdVel;     

    //                 followerState_ = TRACKING;

    //                 startSeacrhingFlag_ = false;

    //                 break;                  

    //             } else {                     

    //                 followerState_ = SEARCHING;

    //                 if( startSeacrhingFlag_ == false){
    //                     startSearching_ = ros::WallTime::now();
    //                 }

    //                 break;     
    //             }
                
    //         }  
    //         case SEARCHING: {

    //             cerr<<" SEARCHING "<<endl;
    //             startSeacrhingFlag_ = true;

    //             auto end = ros::WallTime::now();
    //             auto duration = (end - startSearching_).toSec();
    //             if (duration > continuousSearchDuration_ && startSeacrhingFlag_ == true){
                    
    //                 followerState_ = IDLE;

    //                 startSeacrhingFlag_ = false;

    //                 break;

    //             }

    //             auto cmdVel = cmdVelManager_.createCmdVel(trackingTarget_);
    //             twistCommandPublisher_.publish(cmdVel);

    //             followerState_ = TRACKING;

    //             break; 

    //         }
            
    //     }

    //     publishDebugImg();

    // } else {

    //     cerr<<" erro get robot's pose !!!!!!!  "<<endl;
    // }
        
        
   
      
}


string PersonFollowerSensorFusion::getStringState() {

    switch (followerState_)
    {

        case IDLE: {  

            return  "IDLE";
            break; 

        }
        case TRACKING: {

            return "TRACKING";         
            break;
        }
        
        case SEARCHING: {

            return "SEARCHING";
            break;
        }
    }

    return "";
    
}
bool PersonFollowerSensorFusion::getRobotPoseOdomFrame(geometry_msgs::Pose& robotPose) {


    tf::StampedTransform transform;
    try {

        tfListener_.lookupTransform("odom", "base_link", ros::Time(0), transform);

        robotPose.position.x =   transform.getOrigin().x(); 
        robotPose.position.y =   transform.getOrigin().y();  
        robotPose.position.z =   transform.getOrigin().z();   
        robotPose.orientation.x = transform.getRotation().x();
        robotPose.orientation.y = transform.getRotation().y();	
        robotPose.orientation.z = transform.getRotation().z();        
        robotPose.orientation.w = 1;   

        return true;

    }   
    catch ( const tf::TransformException& e ){

        cerr<<" error transform bewtween odom to base_link "<<endl;
        return false;
    }
}
 

bool PersonFollowerSensorFusion::pickBestTarget() {

    

    ///kalman
    double width = trackingTarget_.state.at<float>(4);
    double height = trackingTarget_.state.at<float>(5);
    trackingTarget_.x_pos_pred = trackingTarget_.state.at<float>(0) ;
    trackingTarget_.y_pos_pred =  trackingTarget_.state.at<float>(1);



    double closestIndex = -1;

    double minDist = 9999;

    cv::Point2d predictedTarget(trackingTarget_.x_pos_pred, trackingTarget_.y_pos_pred); //kalman

    // loop over all targets
    for(int i =0; i < currentFullTargets_.size(); i++ ) {

        /// merged
        if( currentFullTargets_[i].hasCamera_  == true && currentFullTargets_[i].hasLeg_ == true){

            // merged
            double distance =  distanceCalculate(
                cv::Point2d(currentFullTargets_[i].mergedTargetPosition_.point.x, currentFullTargets_[i].mergedTargetPosition_.point.y),
                predictedTarget);

            if( distance < minDist ){
                closestIndex = i;
                minDist = distance;
            }    

         
        } // only camera
        else  if( currentFullTargets_[i].hasCamera_  == true && currentFullTargets_[i].hasLeg_ == false) { 

            double distance =  distanceCalculate(
                cv::Point2d(currentFullTargets_[i].cameraTarget_.position_.point.x, currentFullTargets_[i].cameraTarget_.position_.point.y),
                predictedTarget);

            if( distance < minDist ){
                closestIndex = i;
                minDist = distance;
            }        
                      
        }         
        else { // only leg

            double distance =  distanceCalculate(
                cv::Point2d(currentFullTargets_[i].legTarget_.position_.point.x, currentFullTargets_[i].legTarget_.position_.point.y),
                predictedTarget);

            if( distance < minDist ){
                closestIndex = i;
                minDist = distance;
            }                   
        }
       
    }

    /// lost the targert. use predicted 
    if( minDist > 1.0  ) {

        return false;
        // FullTarget kalmanTarget;
        // kalmanTarget.hasLeg_ = true;
        // kalmanTarget.hasCamera_ = false;
        // kalmanTarget.legTarget_.position_.point.x =  trackingTarget_.x_pos_pred;
        // kalmanTarget.legTarget_.position_.point.y =  trackingTarget_.y_pos_pred;
        
        // kalmanTarget.legTarget_.degAngle_  = calcTargetDegAngle(cv::Point2d(kalmanTarget.legTarget_.position_.point.x,
        //     kalmanTarget.legTarget_.position_.point.y));

        // kalmanTarget.legTarget_.distance_ =  distanceCalculate(cv::Point2d(kalmanTarget.legTarget_.position_.point.x, 
        //     kalmanTarget.legTarget_.position_.point.y),
        //     cv::Point2d(currentRobotPose_.position.x, currentRobotPose_.position.y));

        // trackingTarget_.fullTarget_ = kalmanTarget;
        // trackingTarget_.updateGlobalPositionAndAngle();

        // return true;
    }

    ///if the target found
    if( closestIndex != -1 ){

        trackingTarget_.fullTarget_ = currentFullTargets_[closestIndex];
        trackingTarget_.updateGlobalPositionAndAngle();

        return true;
    }

    return false;

}

double PersonFollowerSensorFusion::getRobotHeading(const geometry_msgs::Pose &pose) const
{

    return atan2((2.0 *
                    (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)),
                    (1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)));
}



bool PersonFollowerSensorFusion::checkIfCanInitTaregt() const  {


    if( currentFullTargets_.size() > 0 ){

        if( currentFullTargets_[0].hasCamera_ == true){

            if( currentFullTargets_[0].cameraTarget_.distance_ < maxDistInitCameraTarget_ ){ //meters
                return true;
            }
            
        }
    }

    return false;

}

void PersonFollowerSensorFusion::detectedObjectsCallback(const object_msgs::ObjectsInBoxes::Ptr &objects)
{    
    currentCameraTargets_.clear();

    cv::Mat depthGrayscale;
    double minVal;
    double maxVal;
    minMaxLoc(currentDepthImg_, &minVal, &maxVal);
    currentDepthImg_.convertTo(depthGrayscale, CV_8UC1, (255 / (maxVal - minVal)));
    cv::Mat distanceTransformImg;
    distanceTransform(depthGrayscale, distanceTransformImg, DIST_L2, 3);
    normalize(distanceTransformImg, distanceTransformImg, 0, 1.0, NORM_MINMAX);


    if (cameraInfoInited_) {

        cv::Mat depthGrayscale;
            double minVal;
            double maxVal;
            minMaxLoc(currentDepthImg_, &minVal, &maxVal);
            currentDepthImg_.convertTo(depthGrayscale, CV_8UC1, (255 / (maxVal - minVal)));


        for (int i = 0; i < objects->objects_vector.size(); i++) {

            if( objects->objects_vector[i].object.object_name != trackingTargetClassName_){
                continue;
            }

            cv::Rect rgb_object_rect(objects->objects_vector[i].roi.x_offset,
               objects->objects_vector[i].roi.y_offset,
               objects->objects_vector[i].roi.width,
               objects->objects_vector[i].roi.height);

            //scaling depth b_box
            cv::Rect depth_bounding_box ;
            double scale_x = double((double)currentDepthImg_.cols / (double) currentRgbImg_.cols );
            double scale_y = double((double)currentDepthImg_.rows / (double) currentRgbImg_.rows );
            depth_bounding_box.width = rgb_object_rect.width * scale_x;
            depth_bounding_box.height = rgb_object_rect.height * scale_y;
            depth_bounding_box.x = rgb_object_rect.x * scale_x;
            depth_bounding_box.y = rgb_object_rect.y * scale_y;            

            cv::Point2d depthPix;
            if( findLargestDepthBlob(depth_bounding_box, distanceTransformImg, depthPix) ) {
                
                geometry_msgs::PointStamped objectPose;         
                if(extractDepthFromBboxObject(depthPix, currentDepthImg_, objectPose)) {                    
                    

                    bool noiseDepth = checkIfDepthIsNoiseByLaser(objectPose);
                    if( noiseDepth){
                        continue;
                    }
                    CameraTarget target;
                    target.position_ = objectPose;
                    target.rgb_bounding_box_ = rgb_object_rect;
                    target.rgbPix_.x = depthPix.x / scale_x ;
                    target.rgbPix_.y = depthPix.y / scale_y ;
                     target.degAngle_   = calcTargetDegAngle(cv::Point2d(target.position_.point.x, target.position_.point.y));
                    target.distance_ =  distanceCalculate(cv::Point2d(target.position_.point.x, target.position_.point.y),
                     cv::Point2d(currentRobotPose_.position.x, currentRobotPose_.position.y));

                    currentCameraTargets_.push_back(target);
                } 

               
            }
                
        }     
     
    }
        
}


bool PersonFollowerSensorFusion::checkIfDepthIsNoiseByLaser(const geometry_msgs::PointStamped& objectPose) {

    int minNumOfPoints = 2;
    
    int numOfPoints = 0;
    
    for (int j = 0; j < currentScanPoints_.size(); j++) {

        double dist = distanceCalculate(cv::Point2d(objectPose.point.x, objectPose.point.y),
                     cv::Point2d(currentScanPoints_[j].x, currentScanPoints_[j].y));
        if( dist < 0.2){
            numOfPoints++;
        }      

        if( numOfPoints >= minNumOfPoints) {
            break;
        }      
    }

    if( numOfPoints < minNumOfPoints ){

        return true;
    }

    return false;

}
double PersonFollowerSensorFusion::distanceCalculate(cv::Point2d p1, cv::Point2d p2)
{
    double x = p1.x - p2.x; //calculating number to square in next step
    double y = p1.y - p2.y;
    double dist;

    dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
    dist = sqrt(dist);

    return dist;
}


bool PersonFollowerSensorFusion::findLargestDepthBlob(const cv::Rect& object_rect,
    const Mat& distanceTransformImg, cv::Point2d& depthPix) {

    cv::Mat croppedBbox = distanceTransformImg(object_rect);
    cv::Point minPoint, maxPoint;
    double minValue;
    double maxValue;
    cv::minMaxLoc(croppedBbox, &minValue,&maxValue, &minPoint, &maxPoint);
    if (maxValue <= 0)
        return false;
    
    double val = maxValue - 0.3;
    cv::Mat binarayImage(croppedBbox.rows, croppedBbox.cols,CV_8UC1, cv::Scalar(0));
    binarayImage.setTo(255, croppedBbox >= val);
    vector<vector<Point> >contours;
    vector<Vec4i>hierarchy;
    int savedContour = -1;
    double maxArea = 0.0;
    // Find the largest contour
    findContours(binarayImage, contours, hierarchy,RETR_TREE, CHAIN_APPROX_SIMPLE, Point());    

    if( contours.size() == 0) {
        return false;
    }
	// find the largest contour
    for (int i = 0; i< contours.size(); i++)
    {
        double area = contourArea(contours[i]);

        if (area > maxArea)
        {
            maxArea = area;
            savedContour = i;
        }

    }
    // get the moments
    vector<cv::Moments> mu(contours.size());
	for( int i = 0; i<contours.size(); i++ ){
        mu[i] = moments( contours[i], false ); 
    }
    // get the centroid of figures.
	vector<Point2f> mc(contours.size());
	for( int i = 0; i< contours.size(); i++){ 
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        
    }

 
    
    depthPix = cv::Point2d(mc[savedContour].x + object_rect.x,mc[savedContour].y + object_rect.y );
    return true;

}


vector<FullTarget> PersonFollowerSensorFusion::createFullTargets() {

    vector<FullTarget> fullTargets;

    double maxDistLegToCameraThreshold = 0.3;

    vector<bool> legsUsed;
    legsUsed.resize(currentLegsTargets_.size());

    //all legs target not allocate to camera tragets
    for(int i =0; i < legsUsed.size(); i++ ){
        legsUsed[i] = false;
    }
    
    //loop over cameras targets
    for (int i = 0; i < currentCameraTargets_.size(); i++)
    {      
        CameraTarget cameraTarget = currentCameraTargets_[i];
        int legIndex = -1;

        double minDist = 99999;
        
        //loop over legs targets
        for (int j = 0; j < currentLegsTargets_.size(); j++) {

            if( legsUsed[j] == false) {
                geometry_msgs::PointStamped legTarget = currentLegsTargets_[j].position_;

                double dist = distanceCalculate(
                    cv::Point2d(legTarget.point.x, legTarget.point.y),
                    cv::Point2d(cameraTarget.position_.point.x, legTarget.point.y));

                if( dist < maxDistLegToCameraThreshold){
                    
                    // if the camera target near leg target, merge them
                    if( dist < minDist) {
                    
                        minDist = dist;
                        legIndex = j;
                    }
                } 
                    
            }        

        }

        FullTarget fullTarget;
        // can merge camera and lidar
        if( legIndex != -1 ){
            
            // allocate leg target
            legsUsed[legIndex] = true;

            /// create full target with camera and leg
            fullTarget.cameraTarget_ = cameraTarget;
            fullTarget.hasCamera_ = true;
            fullTarget.legTarget_ = currentLegsTargets_[legIndex];
            fullTarget.hasLeg_ = true;

            //create merged target
            
            //set merged target location by camera
            fullTarget.mergedTargetPosition_.point.x = 
                (fullTarget.cameraTarget_.position_.point.x + fullTarget.legTarget_.position_.point.x ) / 2;
            fullTarget.mergedTargetPosition_.point.y = 
                (fullTarget.cameraTarget_.position_.point.y + fullTarget.legTarget_.position_.point.y ) / 2;
            //set merged target angle by camera
            fullTarget.mergeTargetAngle_ = fullTarget.legTarget_.degAngle_;
        
        } else {
            //only camera
            fullTarget.cameraTarget_ = cameraTarget;
            fullTarget.hasCamera_ = true;  
            fullTarget.hasLeg_ = false;         
        }

        fullTargets.push_back(fullTarget);

      
    }

    // only legs tragets (without camera)
    for(int i =0; i < legsUsed.size(); i++ ){
        
        if( legsUsed[i] == false) {

            FullTarget fullTarget;
            fullTarget.legTarget_ = currentLegsTargets_[i];
            fullTarget.hasLeg_ = true;
            fullTarget.hasCamera_ = false; 

            //target in front of the robot but camera cant recognize -> noise!  
            if(!(fullTarget.legTarget_.degAngle_ > 120 &&  fullTarget.legTarget_.degAngle_ < 220)){

                if( fullTarget.legTarget_.distance_ > 3){
                    continue;
                }    
            }

            fullTargets.push_back(fullTarget);

            legsUsed[i] = true;
        }
        
    }

    return fullTargets;


}

void PersonFollowerSensorFusion::publishDebugImg(){

    double mapSizeW = MAP_W_METER * RESOULTION;
    double mapSizeH = MAP_W_METER * RESOULTION;

    cv::Point2d center(mapSizeW / 2, mapSizeH / 2);

    cv::Mat debugImg(mapSizeH, mapSizeW, CV_8UC3, cv::Scalar(0));

    circle(debugImg, center, 4, Scalar(0, 255, 0), -1, 8, 0);


    //circle of radius serach
    double dist = 0.2;
    while (dist < MAP_W_METER){ 
        circle(debugImg, center, dist * RESOULTION , Scalar(0, 100, 0), 1, 8, 0);

        cv::Point p(center.x, center.y + (dist * RESOULTION) );
        dist += 0.5;
    }
    
    


    // ROBOT
    geometry_msgs::Pose robotPose;
     cv::Point2d robotPosePix;
    if (getRobotPoseOdomFrame(robotPose)){

         // robot pose odom frame
        robotPosePix = cv::Point2d((robotPose.position.x * RESOULTION) + center.x,
            ( robotPose.position.y * RESOULTION) + center.y);

        circle(debugImg, robotPosePix, 3, Scalar(255,127,80), 5, 8, 0); 

        double heading = getRobotHeading(robotPose);
        cv::Point2d headingPoint(robotPosePix.x + 30 * cos((heading)),
                                    robotPosePix.y + 30 * sin((heading)));
        cv::arrowedLine(debugImg, robotPosePix, headingPoint, Scalar(0,255,0), 2);


        //camera angles
        cv::Point2d leftCmaeraAngle(robotPosePix.x + 100 * cos(heading-(angles::from_degrees(30))),
                                    robotPosePix.y + 100 * sin(heading-(angles::from_degrees(30))));
        //camera angles
        cv::Point2d rightCmaeraAngle(robotPosePix.x + 100 * cos(heading+(angles::from_degrees(30))),
                                    robotPosePix.y + 100 * sin(heading+(angles::from_degrees((30)))));                            
        cv::arrowedLine(debugImg, robotPosePix, leftCmaeraAngle, Scalar(255,255,255), 2); 
        cv::arrowedLine(debugImg, robotPosePix, rightCmaeraAngle, Scalar(255,255,255), 2);   

        // robot only rotation radius                                  
        circle(debugImg, robotPosePix, maxDistOnlyRotation_ * RESOULTION, Scalar(0, 128, 255), 1, 8, 0);      

        // max distance tracking     
        cerr<<" maxDistanceTracking_ " <<maxDistanceTracking_<<endl;
        
        circle(debugImg, robotPosePix, maxDistanceTracking_ * RESOULTION, Scalar(153, 51, 0), 1, 8, 0);      

    }

    
    for(int i =0; i < currentFullTargets_.size(); i++ ) {

        /// merged
        if( currentFullTargets_[i].hasCamera_  == true && currentFullTargets_[i].hasLeg_ == true){

            // merged
            cv::Point2d mergedTarget((currentFullTargets_[i].mergedTargetPosition_.point.x * RESOULTION) + center.x,
                ( currentFullTargets_[i].mergedTargetPosition_.point.y * RESOULTION) + center.y);

            circle(debugImg, mergedTarget, 5, Scalar(0,255,255), 5, 8, 0);    

            //camera : 
            cv::Point2d cameraTarget((currentFullTargets_[i].cameraTarget_.position_.point.x * RESOULTION) + center.x,
                ( currentFullTargets_[i].cameraTarget_.position_.point.y * RESOULTION) + center.y);

            circle(debugImg, cameraTarget, 5, Scalar(255,0,255), -1, 8, 0);
           

            circle(currentRgbImg_, currentFullTargets_[i].cameraTarget_.rgbPix_, 5, Scalar(255,0,255), -1, 8, 0);  
            cv::rectangle(currentRgbImg_, currentFullTargets_[i].cameraTarget_.rgb_bounding_box_, cv::Scalar(0,255,0), 2);

            cv::Point p_angle(currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.tl().x, currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.height - 20);
            cv::Point p_dist(currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.tl().x, currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.height - 40);

   
            putText(currentRgbImg_,  "angle: " + to_string((int)currentFullTargets_[i].mergeTargetAngle_), p_angle,1, 1, Scalar(0,255,255));
            putText(currentRgbImg_,  "dist: " + to_string((int)currentFullTargets_[i].cameraTarget_.distance_), p_dist,1, 1, Scalar(0,255,255));    

            //leg: 
            cv::Point2d legTarget((currentFullTargets_[i].legTarget_.position_.point.x * RESOULTION) + center.x,
                ( currentFullTargets_[i].legTarget_.position_.point.y * RESOULTION) + center.y);

            circle(debugImg, legTarget, 5, Scalar(255,255,0), -1, 8, 0);

          
         
        } // only camera
        else  if( currentFullTargets_[i].hasCamera_  == true && currentFullTargets_[i].hasLeg_ == false) { 

            cv::Point2d cameraTarget((currentFullTargets_[i].cameraTarget_.position_.point.x * RESOULTION) + center.x,
                ( currentFullTargets_[i].cameraTarget_.position_.point.y * RESOULTION) + center.y);

            circle(debugImg, cameraTarget, 5, Scalar(255,0,255), -1, 8, 0);
            // putText(debugImg,  to_string((int) currentFullTargets_[i].cameraTarget_.degAngle_),
            //     cameraTarget,0.2, 1, Scalar(255, 255, 255));

            circle(currentRgbImg_, currentFullTargets_[i].cameraTarget_.rgbPix_, 5, Scalar(255,0,255), -1, 8, 0);  
            cv::rectangle(currentRgbImg_, currentFullTargets_[i].cameraTarget_.rgb_bounding_box_, cv::Scalar(0,0,255), 2);

            cv::Point p_angle(currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.tl().x, currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.height - 20);
            cv::Point p_dist(currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.tl().x, currentFullTargets_[i].cameraTarget_.rgb_bounding_box_.height - 40);
        
            putText(currentRgbImg_,  "angle: " + to_string((int)currentFullTargets_[i].cameraTarget_.degAngle_), p_angle,1, 1, Scalar(0,255,255));
            putText(currentRgbImg_,  "dist: " + to_string((int)currentFullTargets_[i].cameraTarget_.distance_), p_dist,1, 1, Scalar(0,255,255));
        }         
        else { // only leg

            //leg: 
            cv::Point2d legTarget((currentFullTargets_[i].legTarget_.position_.point.x * RESOULTION) + center.x,
                ( currentFullTargets_[i].legTarget_.position_.point.y * RESOULTION) + center.y);

            circle(debugImg, legTarget, 5, Scalar(255,255,0), -1, 8, 0);
        }
       
    }

    // scan
    for (int j = 0; j < currentScanPoints_.size(); j++) {

        cv::Point2d pIMG((currentScanPoints_[j].x * RESOULTION) + center.x,
            (currentScanPoints_[j].y * RESOULTION) + center.y);

        circle(debugImg, pIMG, 1, Scalar(0, 0, 255), -1, 8, 0);
    }
    
    // trackingTarget
    if( followerState_ == TRACKING || followerState_ == SEARCHING){
        
        cv::Point2d trackingTargetPix((trackingTarget_.globalPosition_.point.x * RESOULTION) + center.x,
            ( trackingTarget_.globalPosition_.point.y * RESOULTION) + center.y);
        circle(debugImg, trackingTargetPix, 10, Scalar(255, 255, 255), 2, 8, 0);

        cv::arrowedLine(debugImg, robotPosePix, trackingTargetPix, Scalar(0,255,0), 2);

        if( trackingTarget_.fullTarget_.hasCamera_ ){
            cv::rectangle(currentRgbImg_, 
                trackingTarget_.fullTarget_.cameraTarget_.rgb_bounding_box_, cv::Scalar(0,255,0), 2);

        }
        
    }

    // if( followerState_ == TRACKING || followerState_ == SEARCHING) {

    //     //kalman
        
    //     cv::Point2d kalmanPoint((trackingTarget_.x_pos_pred * RESOULTION) + center.x,
    //         ( trackingTarget_.y_pos_pred  * RESOULTION) + center.y);
    //     circle(debugImg, kalmanPoint, 10, Scalar(255, 50, 120), 2, 8, 0);       
  
    // }
    
   
     

    flip(debugImg,debugImg, 0);

     if( followerState_ == TRACKING ) {
        
        putText(debugImg,  "target angle " + to_string( trackingTarget_.globalDegAngle_),
            cv::Point(50,50), 0.2, 1, Scalar(255, 255, 255));

        putText(debugImg,  "target distance " + to_string((int) trackingTarget_.globalDistance_),
            cv::Point(50,100), 0.2, 1, Scalar(255, 255, 255));  

     }
 

     //current state
    putText(debugImg,  getStringState(), cv::Point(debugImg.cols / 2 ,50),1, 2, Scalar(0,255,0));

    // imshow("currentRgbImg_",currentRgbImg_);
    // imshow("debugImg",debugImg);
    // waitKey(1);

    sensor_msgs::ImagePtr msg = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugImg).toImageMsg();
    
    debugImgPublisher_.publish(msg);


    
}
