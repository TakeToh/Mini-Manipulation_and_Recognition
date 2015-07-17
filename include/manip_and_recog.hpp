#ifndef __MANIP_AND_RECOG__
#define __MANIP_AND_RECOG__

#include "ros/ros.h"

// Include Actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>

// Message that was deal with ORK_SERVER
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#include <object_recognition_msgs/ObjectRecognitionActionResult.h>
// Server Result Class
#include <object_recognition_msgs/RecognizedObjectArray.h>

// To deal with Messeages 
#include <object_recognition_msgs/GetObjectInformation.h>
#include <object_recognition_msgs/ObjectType.h>
#include <object_recognition_msgs/ObjectInformation.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <iostream>

// To drive KOBUKI
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/String.h>

// To Move ARM
#include <manip_and_recog/ArmPose.h>
#include <manip_and_recog/Servo.h>

// DEBUG OPTION
#define DEBUG

// _____________MACRO_____________

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0)
#define RAD2DEG(RAD) ( 180.0 * (RAD) / (M_PI))

template < typename TYPE, int SIZE>
int getSize(const TYPE (&x)[SIZE]){	return SIZE;	}

//	 _____________ENUM DEFINE_____________ 
enum PHASE_STATE
{
	STAGE_BEGIN =0,
	STAGE1 = 1,
	STAGE2 = 2,
	STAGE3 = 3,
	STAGE4 = 4,
	STAGE_END = 99,
	STAGE_TEST = 100
};

enum ROBO_CONDITION
{
	SAFE=1,
	MISS=-1,
	DANGER=119,
};

//_____________GROBAL_INSTANCE_____________ 

struct POSE
{
	double x;
	double y;
	double t;
	double z;
};

//_____________ROBOT_CLASS_____________
class Object
{
private:
	POSE pose;				// current position
	POSE ex_pose;			// previous positon

public:
	Object();
	void SetPosition(object_recognition_msgs::RecognizedObject ob);
	POSE GetPosition();
	void SpeakPosition();
	bool CheckObjectData();
	void BackUpPose(POSE p);
};

//_____________ROBOT_CLASS_____________
class Robot{
private:
	//class
	ros::NodeHandle node;
	ros::Subscriber odom_sub;		// Getting KOBUKI odometry
	ros::Publisher vel_pub;		//Sending velocity to KOBUK
	ros::ServiceClient ArmPointclient;		//In order to execute Servo motor
	ros::ServiceClient Headclient;
	ros::Subscriber OutputPub;
	
	//Action library for object recognition kitchen
	actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> *ac;
	
	object_recognition_msgs::RecognizedObjectArray result;		// Get result date from actionlib
	geometry_msgs::Point pose;									// Information about position
	geometry_msgs::Point ex_pose;									// Information about position

	double theta;
	double ex_theta;
	PHASE_STATE state;

	double distance;
	double ex_distance;

public:
	void init();
	void HeadUp();
	void GetActionlibResult(const object_recognition_msgs::RecognizedObjectArray& r);
	object_recognition_msgs::RecognizedObjectArray ReturnActionResult(void);
	void CallBackOutput(const std_msgs::String& voice);
	void Move(double _vx, double _vy,double _vtheta);
	void Stop();
	void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
	PHASE_STATE getState();
	void setState( PHASE_STATE ps);
	void SpeakPosition();
	geometry_msgs::Point GetPosition();
	double GetTheta(void);

	//	P H A S E____F U N C T I O N
	ROBO_CONDITION AimToObject(void);
	ROBO_CONDITION GetDataObject(void);
	ROBO_CONDITION CatchObject(void);
};

#endif