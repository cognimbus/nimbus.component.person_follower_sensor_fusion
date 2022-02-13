/*
 * bbox_convertor_node.cpp
 *
 *  Created on: May 13, 2019
 *      Author: Igor Makhtes <igor@cogniteam.com>
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <opencv2/highgui/highgui.hpp>
#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string

#include <ros/ros.h>
#include <object_msgs/ObjectsInBoxes.h>

//for jetson-inference detection
#include <vision_msgs/Detection2DArray.h>


using namespace std;

class ObjectsMsgConvertor {

public:

    ObjectsMsgConvertor(){

        detectedVisionMsgObjectsSubscriber_ = node_.subscribe("/vision_msg_detected_objects", 1, 
            &ObjectsMsgConvertor::detectedObjectsCallback, this);

        openVinoObjectsPub_ = 
            node_.advertise<object_msgs::ObjectsInBoxes>("openvino_toolkit/detected_objects",1);

    }

    ~ObjectsMsgConvertor(){}

public:

    void detectedObjectsCallback(const vision_msgs::Detection2DArray::Ptr& objects) {
        

        object_msgs::ObjectsInBoxes openVinoObjects;

        openVinoObjects.objects_vector.resize(objects->detections.size());

        for(int i = 0; i < objects->detections.size(); i++ ) {

            object_msgs::ObjectInBox  objectInBox; 
            objectInBox.roi.width = objects->detections[i].bbox.size_x;
            objectInBox.roi.height = objects->detections[i].bbox.size_y;
            objectInBox.object.object_name = convertIdToType(objects->detections[i].results[0].id);
            objectInBox.roi.y_offset = 
                objects->detections[i].bbox.center.y - (objects->detections[i].bbox.size_y / 2 );
            objectInBox.roi.x_offset = 
                objects->detections[i].bbox.center.x - (objects->detections[i].bbox.size_x / 2 );

            openVinoObjects.objects_vector[i] = objectInBox;
        }

        openVinoObjectsPub_.publish(openVinoObjects);
    }

    string  convertIdToType(int id){

        if (id == 1){
            return "person";
        }
        return std::to_string(id);
    }
private:

    ros::NodeHandle node_;
    
    ros::Subscriber detectedVisionMsgObjectsSubscriber_;

    ros::Publisher openVinoObjectsPub_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "bbox_convertor_node");
  
    ObjectsMsgConvertor objectsMsgConvertor;

    ros::spin();
    return 0;
}
