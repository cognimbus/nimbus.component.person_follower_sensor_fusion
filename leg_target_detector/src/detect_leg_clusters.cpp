/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
// ROS 
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>


// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/ml.hpp>

// Local headers
#include <leg_target_detector/laser_processor.h>
#include <leg_target_detector/cluster_features.h>

// Custom messages
#include <leg_target_detector/Leg.h>
#include <leg_target_detector/LegArray.h>



using namespace std;
using namespace cv;


struct PersonCluster
{
  vector<cv::Point2d> points_; // tow-legs

  cv::Point2d centroid_;
};

/**
  * @brief Comparison class to order Legs according to their relative distance to the laser scanner
  */
  class CompareLegs
  {
  public:
      bool operator ()(const leg_target_detector::Leg &a, const leg_target_detector::Leg &b)
      {
          float rel_dist_a = pow(a.position.x*a.position.x + a.position.y*a.position.y, 1./2.);
          float rel_dist_b = pow(b.position.x*b.position.x + b.position.y*b.position.y, 1./2.);          
          return rel_dist_a < rel_dist_b;
      }
  };

/**
* @brief Detects clusters in laser scan with leg-like shapes
*/
class DetectLegClusters
{
public:
  /**
  * @brief Constructor
  */
  DetectLegClusters():
  scan_num_(0),
  num_prev_markers_published_(0)
  {  
    // Get ROS parameters  
    std::string forest_file;
    std::string scan_topic;
    if (!nh_.getParam("forest_file", forest_file))
      ROS_ERROR("ERROR! Could not get random forest filename");
    nh_.param("scan_topic", scan_topic, std::string("scan"));
    nh_.param("fixed_frame", fixed_frame_, std::string("odom"));
    nh_.param("detection_threshold", detection_threshold_, -1.0);
    nh_.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
    nh_.param("min_points_per_cluster", min_points_per_cluster_, 3);                
    nh_.param("max_detect_distance", max_detect_distance_, 10.0);   
    nh_.param("marker_display_lifetime", marker_display_lifetime_, 0.2);   
    nh_.param("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);
    nh_.param("max_detected_clusters", max_detected_clusters_, -1);

    // Print back
    ROS_INFO("forest_file: %s", forest_file.c_str());
    ROS_INFO("scan_topic: %s", scan_topic.c_str());
    ROS_INFO("fixed_frame: %s", fixed_frame_.c_str());
    ROS_INFO("detection_threshold: %.2f", detection_threshold_);
    ROS_INFO("cluster_dist_euclid: %.2f", cluster_dist_euclid_);
    ROS_INFO("min_points_per_cluster: %d", min_points_per_cluster_);
    ROS_INFO("max_detect_distance: %.2f", max_detect_distance_);    
    ROS_INFO("marker_display_lifetime: %.2f", marker_display_lifetime_);
    ROS_INFO("use_scan_header_stamp_for_tfs: %d", use_scan_header_stamp_for_tfs_);    
    ROS_INFO("max_detected_clusters: %d", max_detected_clusters_);    

    // Load random forst
    forest = cv::ml::StatModel::load<cv::ml::RTrees>(forest_file);
    feat_count_ = forest->getVarCount();

    latest_scan_header_stamp_with_tf_available_ = ros::Time::now();

    // ROS subscribers + publishers
    scan_sub_ =  nh_.subscribe(scan_topic, 10, &DetectLegClusters::laserCallback, this);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    detected_leg_clusters_pub_ = nh_.advertise<leg_target_detector::LegArray>("detected_leg_clusters", 20);

    targetsPosesPub_ = 
      nh_.advertise<geometry_msgs::PoseArray>("/legs_targets", 10);

  }

private:
  tf::TransformListener tfl_;

  cv::Ptr< cv::ml::RTrees > forest = cv::ml::RTrees::create();

  int feat_count_;

  ClusterFeatures cf_;

  int scan_num_;
  bool use_scan_header_stamp_for_tfs_;
  ros::Time latest_scan_header_stamp_with_tf_available_;

  ros::NodeHandle nh_;
  ros::Publisher markers_pub_;
  ros::Publisher detected_leg_clusters_pub_;
  ros::Publisher targetsPosesPub_;

  ros::Subscriber scan_sub_;

  std::string fixed_frame_;
  
  double detection_threshold_;
  double cluster_dist_euclid_;
  int min_points_per_cluster_;  
  double max_detect_distance_;
  double marker_display_lifetime_;
  int max_detected_clusters_;

  int num_prev_markers_published_;


  /**
  * @brief Clusters the scan according to euclidian distance, 
  *        predicts the confidence that each cluster is a human leg and publishes the results
  * 
  * Called every time a laser scan is published.
  */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {         
    laser_processor::ScanProcessor processor(*scan); 
    processor.splitConnected(cluster_dist_euclid_);        
    processor.removeLessThan(min_points_per_cluster_);    

    // OpenCV matrix needed to use the OpenCV random forest classifier
    cv::Mat tmp_mat(1, feat_count_, CV_32FC1); 
    
    leg_target_detector::LegArray detected_leg_clusters;
    detected_leg_clusters.header.frame_id = scan->header.frame_id;
    detected_leg_clusters.header.stamp = scan->header.stamp;

    // Find out the time that should be used for tfs
    bool transform_available;
    ros::Time tf_time;
    // Use time from scan header
    if (use_scan_header_stamp_for_tfs_)
    {
      tf_time = scan->header.stamp;

      try
      {
        tfl_.waitForTransform(fixed_frame_, scan->header.frame_id, tf_time, ros::Duration(1.0));
        transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
      }
      catch(tf::TransformException ex)
      {
        ROS_INFO("Detect_leg_clusters: No tf available");
        transform_available = false;
      }
    }
    else
    {
      // Otherwise just use the latest tf available
      tf_time = ros::Time(0);
      transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
    }
    
    // Store all processes legs in a set ordered according to their relative distance to the laser scanner
    std::set <leg_target_detector::Leg, CompareLegs> leg_set;
    if (!transform_available)
    {
      ROS_INFO("Not publishing detected leg clusters because no tf was available");
    }
    else // transform_available
    {
      // Iterate through all clusters
      for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
       cluster != processor.getClusters().end();
       cluster++)
      {   
        // Get position of cluster in laser frame
        tf::Stamped<tf::Point> position((*cluster)->getPosition(), tf_time, scan->header.frame_id);
        float rel_dist = pow(position[0]*position[0] + position[1]*position[1], 1./2.);
        
        // Only consider clusters within max_distance. 
        if (rel_dist < max_detect_distance_)
        {
          // Classify cluster using random forest classifier
          std::vector<float> f = cf_.calcClusterFeatures(*cluster, *scan);
          for (int k = 0; k < feat_count_; k++)
            tmp_mat.at<float>(k) = (float)(f[k]);
          
          cv::Mat result;
          forest->getVotes(tmp_mat, result, 0);
          int positive_votes = result.at<int>(1, 1);
          int negative_votes = result.at<int>(1, 0);
          float probability_of_leg = positive_votes / static_cast<double>(positive_votes + negative_votes);

          // Consider only clusters that have a confidence greater than detection_threshold_                 
          if (probability_of_leg > detection_threshold_)
          { 
            // Transform cluster position to fixed frame
            // This should always be succesful because we've checked earlier if a tf was available
            bool transform_successful_2;
            try
            {
              tfl_.transformPoint(fixed_frame_, position, position);
              transform_successful_2 = true;
            }
            catch (tf::TransformException ex)
            {
              ROS_ERROR("%s",ex.what());
              transform_successful_2 = false;
            }

            if (transform_successful_2)
            {  
              // Add detected cluster to set of detected leg clusters, along with its relative position to the laser scanner
              leg_target_detector::Leg new_leg;
              new_leg.position.x = position[0];
              new_leg.position.y = position[1];
              new_leg.confidence = probability_of_leg;
              leg_set.insert(new_leg);
            }
          }
        }
      }

      

    }    


    std::vector<PersonCluster> perosnsClusters = createLegTragets(leg_set);

    geometry_msgs::PoseArray  posearray;
    posearray.header.stamp = ros::Time::now(); 
    posearray.header.frame_id = fixed_frame_;

    for (int i = 0; i < perosnsClusters.size(); i++) {

        geometry_msgs::Pose p; 
        p.position.x = perosnsClusters[i].centroid_.x;
        p.position.y = perosnsClusters[i].centroid_.y;
        p.position.z = 0.0;
        posearray.poses.push_back(p);      
    }

    targetsPosesPub_.publish(posearray);

   
  }


   std::vector<PersonCluster> createLegTragets(std::set <leg_target_detector::Leg, CompareLegs>& leg_set){

    std::vector<PersonCluster> perosnsClusters;

    for (std::set<leg_target_detector::Leg>::iterator it = leg_set.begin(); it != leg_set.end(); ++it) {
      
      leg_target_detector::Leg leg = *it;

      if( leg.confidence > 0.15){
        cv::Point2d legPoint(leg.position.x,leg.position.y);
        addLegToPersonClusters(legPoint, perosnsClusters);      
      
      }
     
    }

    removeExtraLegsFromTarget(perosnsClusters);

    return perosnsClusters;

  }

  void removeExtraLegsFromTarget(std::vector<PersonCluster> &perosnsClusters) {

      vector<PersonCluster> filteredClusters;

      for (int i = 0; i < perosnsClusters.size(); i++)
      {
          if( perosnsClusters[i].centroid_.x == 0 && perosnsClusters[i].centroid_.y == 0) {
            continue;
          }
          if (perosnsClusters[i].points_.size() > 1 && perosnsClusters[i].points_.size() <=2 )
          {
            filteredClusters.push_back(perosnsClusters[i]);
          }
      }

      perosnsClusters = filteredClusters;
  }

  void addLegToPersonClusters(const cv::Point2d &legPoint,
      vector<PersonCluster> & personClusters)
  {

      if (personClusters.size() == 0)
      {
          PersonCluster cluster;
          cluster.points_.push_back(legPoint);
          cluster.centroid_ = cv::Point2d(legPoint);
          personClusters.push_back(cluster);
          return;
      }
      // try to find cluster
      for (int i = 0; i < personClusters.size(); i++)
      {

          for (int j = 0; j < personClusters[i].points_.size(); j++)
          {

              double dist = distanceCalculate(legPoint, personClusters[i].points_[j]);
              if (dist < 0.7 && personClusters[i].points_.size() < 3)
              {
                  personClusters[i].points_.push_back(legPoint);

                  personClusters[i].centroid_ = calcClusterCentroid(personClusters[i].points_);
                  return;
              }
          }
      }

      //open a new cluster
      PersonCluster personCluster;
      personCluster.points_.push_back(legPoint);
      personCluster.centroid_ = cv::Point2d(legPoint);
      personClusters.push_back(personCluster);
  }  

  cv::Point2d calcClusterCentroid(const vector<cv::Point2d> &points)
  {

      cv::Point2d centroid(0, 0);

      for (int j = 0; j < points.size(); j++)
      {

          centroid.x += points[j].x;
          centroid.y += points[j].y;
      }

      centroid.x = centroid.x / points.size();
      centroid.y = centroid.y / points.size();

      return centroid;
  }

  double distanceCalculate(cv::Point2d p1, cv::Point2d p2)
  {
      double x = p1.x - p2.x; //calculating number to square in next step
      double y = p1.y - p2.y;
      double dist;

      dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
      dist = sqrt(dist);

      return dist;
  }

  

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_leg_clusters");
  DetectLegClusters dlc;
  ros::spin();
  return 0;
}

