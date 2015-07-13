#include "manip_and_recog.hpp"

///////////////////////////////////////////////////////////////////
///					G R O B A L __ I N S T A N C E 
///////////////////////////////////////////////////////////////////
int ObjectsNum =0;					//The number is CLASS'Oject' number
Object object[10];					//Object class
PHASE_STATE TEST_MAX = STAGE2;			//if TEST_MAX's num finished, TEST_MAX=STATE_END


///////////////////////////////////////////////////////////////////
///					C L A S S ""R O B O T "" initilization 
///////////////////////////////////////////////////////////////////

void Robot::init()
{
	odom_sub = node.subscribe("odom",100,&Robot::odomCallback,this);		// Getting KOBUKI odometry
	vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",100);		//Sending velocity to KOBUKI

	//Using actionlib in order to execute simple action server's client
	//trueのところをfalseに変えると実行されないことがわかった
	ac = new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>("recognize_objects", true);
	ac->waitForServer();

	//In order to execute Servo motor
	ArmPointclient = node.serviceClient< manip_and_recog::ArmPose >("arm_pose");
}

PHASE_STATE Robot::getState()
{
	return state;
}

void Robot::setState( PHASE_STATE ps)
{
	state = ps;
}


///////////////////////////////////////////////////////////////////
///				K O B U K I __ M O V E M E N T
///////////////////////////////////////////////////////////////////

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
	ex_pose = pose;
	ex_theta = theta;

   	pose.x = odom->pose.pose.position.x;
    pose.y = odom->pose.pose.position.y;
    pose.z = 0;

    geometry_msgs::Quaternion quat;
    quat.x = odom->pose.pose.orientation.x;
    quat.y = odom->pose.pose.orientation.y;
    quat.z = odom->pose.pose.orientation.z;
    quat.w = odom->pose.pose.orientation.w;

    theta = tf::getYaw(quat);
}

void Robot::SpeakPosition()
{
	std::cout << "ROBOT POSITION" << std::endl;
	std::cout << "Current Positon  " << " x: " << pose.x << " y: " << pose.y;
	std::cout << " z: " << pose.z << " theta: " << theta << std::endl;


	std::cout << "Previous Positon  " << " x: " << ex_pose.x << " y: " << ex_pose.y;
	std::cout << " z: " << ex_pose.z << " theta: " << ex_theta << std::endl;
}

void Robot::Move(double _vx=0, double _vy=0,double _vtheta=0)
{
	geometry_msgs::Twist cmd_vel;		// speed parametor

    cmd_vel.linear.x = _vx;
    cmd_vel.linear.y = _vy;
    cmd_vel.angular.z = _vtheta;

    vel_pub.publish(cmd_vel);
}

void Robot::Stop()
{
	geometry_msgs::Twist cmd_vel;		// speed parametor

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;

    vel_pub.publish(cmd_vel);
}

geometry_msgs::Point Robot::GetPosition()
{	
	return pose;
}

double Robot::GetTheta(void)
{
	return theta;
}

///////////////////////////////////////////////////////////////////
///			A C T I O N S E R V E R __ B E H A V I O R
///////////////////////////////////////////////////////////////////
void Robot::GetActionlibResult(const object_recognition_msgs::RecognizedObjectArray& r)
{
	if(&r.cooccurrence[0] !=NULL){
		result =r;	
	}else{
		ROS_INFO("Actionlib Result has no data\n");
	}
}

object_recognition_msgs::RecognizedObjectArray Robot::ReturnActionResult(void)
{
	if(result.cooccurrence[0]!=0){
		return result;
	}else{
		ROS_INFO("Actionlib Result has no data\n");
	}
}

///////////////////////////////////////////////////////////////////
///			C L A S S "" O B J E C T ""
///////////////////////////////////////////////////////////////////
Object::Object()
{

}

void Object::BackUpPose(POSE p){
	ex_pose.x = p.x;
	ex_pose.y = p.y;
	ex_pose.z = p.z;
	ex_pose.t = p.t;
}


void Object::SetPosition(object_recognition_msgs::RecognizedObject ob)
{
	// Getting position x , y , z
	// Transform  ORK_coodinate_system  World_coodinate_system
	
	BackUpPose(pose);
			
	pose.x = ob.pose.pose.pose.position.x;
	pose.y = ob.pose.pose.pose.position.z;
	pose.z = -1*ob.pose.pose.pose.position.y;

	// TransForm quaternion to revoltuion
	geometry_msgs::Quaternion quat;
	quat.x = ob.pose.pose.pose.orientation.x;
    quat.y = ob.pose.pose.pose.orientation.z;
    quat.z = -1*ob.pose.pose.pose.orientation.y;
    quat.w = ob.pose.pose.pose.orientation.w;

	pose.t = tf::getYaw(quat);

}

POSE Object::GetPosition()
{
	return pose;
}

void Object::SpeakPosition()
{
	std::cout << "Object Data" << std::endl;
	std::cout << "Current Positon  " << " x: " << pose.x << " y: " << pose.y;
	std::cout << " z: " << pose.z << " theta: " << pose.t << std::endl;


	std::cout << "Previous Positon  " << " x: " << ex_pose.x << " y: " << ex_pose.y;
	std::cout << " z: " << ex_pose.z << " theta: " << ex_pose.t << std::endl;
}

bool Object::CheckObjectData()
{
	if(pose.x+pose.y+pose.z!=0)
	{
		return true;
	}else{
		return false;
	}

}

///////////////////////////////////////////////////////////////////
///			P H A S E _ B E G I N	"" G E T   O B J E C T S   P O S I T I O N""
///			1st 	Sending goal data to actionserver
///			2nd		Getting result data about objects from the actionserver
///			3rd		Getting data copied to object class
///			4th		LOOP 10 times
///////////////////////////////////////////////////////////////////
ROBO_CONDITION Robot::GetDataObject(void)
{
	static int loop=0;

	//1st
	object_recognition_msgs::ObjectRecognitionGoal goal;
	ac->sendGoal(goal);

	bool finished_before_timeout = ac->waitForResult(ros::Duration(10.0));
	object_recognition_msgs::RecognizedObjectArray result;
	object_recognition_msgs::RecognizedObject *ob_;

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac->getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());

	    try
	    {
	    	if(&ac->getResult()->recognized_objects !=0 && &ac->getResult()->recognized_objects.objects[0]){
			    //2nd
			    result = ac->getResult()->recognized_objects;
				//std::cout << "result=" << result << std::endl;

				ob_ = &result.objects[0];
				//std::cout << "result2=" << result << std::endl;
			}else{
				ROS_INFO("THROW EXCEPTION ERROR");
				std::string e = "ERROR";
				throw e;
			}
	    }
	    catch(std::string e){
			 ROS_INFO("This program access Adrress0x0.");
			 return DANGER;
		}

	}else{
	    ROS_INFO("Action did not finish before the time out.");
	    return MISS;
	}

	//3rd
	// Getting Result Data NumberPOSE Object::GetPosition()

	ObjectsNum = sizeof(*ob_) / sizeof(object_recognition_msgs::RecognizedObject);

	// Getting Data send to Class'Object'
	for(int i=0; i<ObjectsNum && i < 10 ; i++)
	{
		std::cout << "Object[" << i <<"] is inputed" << std::endl;
		object[i].SetPosition(ob_[i]);
	}

	if(loop==10){
		state = STAGE1;
		return SAFE;
	}else{
		loop++;
	}
}

///////////////////////////////////////////////////////////////////
///			P H A S E 1		"" A I M  T O  K O B U K I ""
///			1st 	Getting KOBUKI's position
///			2nd		
///////////////////////////////////////////////////////////////////
ROBO_CONDITION Robot::AimToObject()
{
	static int loop=0;

	// Debug Object
	int i=0;

	// INSTANCE
	POSE ob_pose;
	ob_pose = object[i].GetPosition();

	SpeakPosition();

	double AngleAim = atan( ob_pose.x / ob_pose.y) *-1;
	std::cout << "Angle Aim = " << AngleAim << std::endl;


	if(AngleAim-theta  > 0.1){
		std::cout <<"positive diff= " <<AngleAim-theta<<std::endl;
		Move(0,0,+0.3);
		//ROS_INFO();
		return MISS;
	}else if(AngleAim-theta  < -0.1){
		std::cout <<"negative diff= " <<AngleAim-theta<<std::endl;
		Move(0,0,-0.3);
		return MISS;
	}else{
		Stop();
		return SAFE;
	}

	/*
	if(loop==10){
		std::cout <<" LOOP LIMIT " << std::endl;
		state = STAGE2;
		return SAFE;
	}else{
		loop++;
	}*/
}



///////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
	ROS_INFO("MANIP_AND_RECOGNITION   S T A R T");
	ros::init(argc, argv, "manip_and_recog");

	Robot robot;
	
	robot.init();

	ros::Rate loop_rate(20);   
		
	int loop;

	PHASE_STATE STAGE_NUMBER=STAGE_BEGIN;				//		PHASE NUMBER
	robot.setState( STAGE_NUMBER );

	ROBO_CONDITION result;				


	while(ros::ok()){
		switch( robot.getState() ){
			std::cout<<robot.getState()<<std::endl;
	    	case STAGE_BEGIN:	result=robot.GetDataObject();		break;
	    	case STAGE1:		result=robot.AimToObject();		break;


	    	case STAGE_END:		ROS_INFO("STATE_END");			break;
	    }

	    // LOOP num is upper 10 -> sleep
	    if(loop==10){
	    	loop_rate.sleep();
    		ros::spinOnce();
    		loop=0;
    		continue;
	    }

	    if(result==DANGER){
	    	ROS_INFO("PHASE_No.%d is DANGER\n",STAGE_NUMBER);

	    }else{
	    	//if(robot.getState() == STAGE_BEGIN){
			for(int i=0; i<ObjectsNum && i < 10 ; i++){
				object[i].SpeakPosition();
	    	}
	    	//}
		  
		    //debug STAGE_LIMITER
		    if( robot.getState() == TEST_MAX){
	    		STAGE_NUMBER = STAGE_END;
	    	}
	    }
	    
	    loop++;
	    loop_rate.sleep();
		ros::spinOnce();
	}
    ROS_INFO("FINISHED");

	return 0;
}
