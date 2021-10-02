#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include "raw_skeleton/update_user.h"
#include "sstream"
#include "string"

#define ACTIVATE_MACROS 0
#if ACTIVATE_MACROS
    #define HEAD transform[0]
    #define NECK transform[1]
    #define RIGHT_SHOULDER transform[2]
    #define LEFT_SHOULDER transform[3]
    #define RIGHT_ELBOW transform[4]
    #define LEFT_ELBOW transform[5]
    #define RIGHT_HAND transform[6]
    #define LEFT_HAND  transform[7]
    #define TORSO transform[8]
    #define RIGHT_HIP transform[9]
    #define LEFT_HIP transform[10]
    #define RIGHT_KNEE transform[11]
    #define LEFT_KNEE transform[12]
    #define RIGHT_FOOT transform[13]
    #define LEFT_FOOT transform[14]
#endif
// Function Prototypes
void updateBodyJoints(geometry_msgs::Vector3 bodyJoints[15],tf::StampedTransform transform[15]);
bool update_user_id(raw_skeleton::update_user::Request &req, 
                    raw_skeleton::update_user::Response &res);

//Global variables
std::stringstream buffer;
std_msgs::String bodyJointArr;
std::string base_frame="/tracker_depth_frame";
std::string human_base_frame="/tracker/user_1";
bool transformation_flag=false;
float transformation_duration=0.2;

int main(int argc, char** argv){
  ros::init(argc, argv, "skeleton_tf_listener");
  ros::NodeHandle node;
  
  ros::ServiceServer s = node.advertiseService("/raw_skeleton/update_user_id",&update_user_id);
  ros::Publisher bodyJointAray_pub = node.advertise<std_msgs::String>("openni2_camera_node/bodyJointArr", 10);

  tf::TransformListener listener;
  geometry_msgs::Vector3 bodyJoints[15];
  tf::StampedTransform transform[15];
  ros::Rate rate(10.0);
  while (node.ok()){
    if(transformation_flag==true){
        try{
          listener.waitForTransform( base_frame, human_base_frame + "/head",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/head",ros::Time(0), transform[0]);
          listener.waitForTransform( base_frame, human_base_frame + "/neck",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/neck",ros::Time(0), transform[1]);
          listener.waitForTransform( base_frame, human_base_frame + "/right_shoulder",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/right_shoulder",ros::Time(0), transform[2]);
          listener.waitForTransform( base_frame, human_base_frame + "/left_shoulder",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/left_shoulder",ros::Time(0), transform[3]);
          listener.waitForTransform( base_frame, human_base_frame + "/right_elbow",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/right_elbow",ros::Time(0), transform[4]);
          listener.waitForTransform( base_frame, human_base_frame + "/left_elbow",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/left_elbow",ros::Time(0), transform[5]);
          listener.waitForTransform( base_frame, human_base_frame + "/right_hand",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/right_hand",ros::Time(0), transform[6]);
          listener.waitForTransform( base_frame, human_base_frame + "/left_hand",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/left_hand",ros::Time(0), transform[7]);
          listener.waitForTransform( base_frame, human_base_frame + "/torso",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/torso",ros::Time(0), transform[8]);
          listener.waitForTransform( base_frame, human_base_frame + "/right_hip",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/right_hip",ros::Time(0), transform[9]);
          listener.waitForTransform( base_frame, human_base_frame + "/left_hip",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/left_hip",ros::Time(0), transform[10]);
          listener.waitForTransform( base_frame, human_base_frame + "/right_knee",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/right_knee",ros::Time(0), transform[11]);
          listener.waitForTransform( base_frame, human_base_frame + "/left_knee",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/left_knee",ros::Time(0), transform[12]);
          listener.waitForTransform( base_frame, human_base_frame + "/right_foot",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/right_foot",ros::Time(0), transform[13]);
          listener.waitForTransform( base_frame, human_base_frame + "/left_foot",ros::Time(0),ros::Duration(transformation_duration));
          listener.lookupTransform(  base_frame, human_base_frame + "/left_foot",ros::Time(0), transform[14]);
          
          updateBodyJoints(bodyJoints,transform);
          bodyJointAray_pub.publish(bodyJointArr);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          transformation_flag=false;
          continue;
        }
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  return 0;
};
void updateBodyJoints(geometry_msgs::Vector3 bodyJoints[15],tf::StampedTransform transform[15]){
    buffer.str("");
    buffer<<"1";
    for(int i=0;i<15;i++){
        bodyJoints[i].x=int(1000*transform[i].getOrigin().x());
        bodyJoints[i].y=int(1000*transform[i].getOrigin().y());
        bodyJoints[i].z=int(1000*transform[i].getOrigin().z());
        buffer<<","<<bodyJoints[i].x<<","<<bodyJoints[i].y<<","<<bodyJoints[i].z;
    }
    bodyJointArr.data=buffer.str();
}

bool update_user_id(raw_skeleton::update_user::Request &req,
                    raw_skeleton::update_user::Response &res)
{  
  try{
    human_base_frame="/tracker/user_" + req.user_id;
    ROS_INFO_STREAM("Target being tracked: " << human_base_frame);
    transformation_flag=req.track_user;
  }
  catch(ros::Exception &ex){
    ROS_ERROR("%s",ex.what());
    res.success=false;
    return false;
  }
  res.success=true;
  return true;
}
