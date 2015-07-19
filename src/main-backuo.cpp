#include "manip_and_recog.hpp"

///////////////////////////////////////////////////////////////////
///					G R O B A L __ I N S T A N C E 
///////////////////////////////////////////////////////////////////
int ObjectsNum =0;					//The number is CLASS'Oject' number
Object object[10];					//Object class
PHASE_STATE TEST_MAX = STAGE3;			//if TEST_MAX's num finished, TEST_MAX=STATE_END


///////////////////////////////////////////////////////////////////
///					D E F I N E 
///////////////////////////////////////////////////////////////////
#define PHASE1_LOOP 10


///////////////////////////////////////////////////////////////////
///					C L A S S ""R O B O T "" initilization 
///////////////////////////////////////////////////////////////////

bool CallMini = false;

void Robot::CallBackOutput(const std_msgs::String& voice)
{
	std_msgs::String voice_command = voice;
	std::cout << "getting data=" << voice << std::endl;

   if(voice_command.data == "MINI"){
		CallMini = true;
	}
}

void Robot::init()
{
	odom_sub = node.subscribe("odom",100,&Robot::odomCallback,this);		// Getting KOBUKI odometry
	vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",100);		//Sending velocity to KOBUKI

	//Using actionlib in order to execute simple action server's client
	//trueのところをfalseに変えると実行されないことがわかった
	ac = new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>("recognize_objects", true);
	ac->waitForServer();

	//In order to execute Servo motor
	Armclient = node.serviceClient< manip_and_recog::ArmPose >("arm_pose");
	OutputPub = node.subscribe("/recognizer/output",100,&Robot::CallBackOutput,this);		
	
}

void Robot::HeadUp()
{
	manip_and_recog::ArmPose HeadSrv;
	HeadSrv.request.x=1010;
	HeadSrv.request.y=1010;
	HeadSrv.request.motion="head";
	if(Armclient.call(HeadSrv)){
		ROS_INFO("______HEAD_POSITON_STATISC___");
	}else{
		ROS_INFO("HEAD_POSITON_FAULT");
	}
	ROS_INFO("ROBOT INIT FINISHED");
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
	ex_distance = distance;

   	pose.x = odom->pose.pose.position.x*1000;		//
    pose.y = odom->pose.pose.position.y*1000;
    pose.z = 0;

    geometry_msgs::Quaternion quat;
    quat.x = odom->pose.pose.orientation.x;
    quat.y = odom->pose.pose.orientation.y;
    quat.z = odom->pose.pose.orientation.z;
    quat.w = odom->pose.pose.orientation.w;

    theta = tf::getYaw(quat);

    distance = sqrt(pose.x*pose.x+pose.y*pose.y);

#ifdef DEBUG
    std::cout << "_________KOBUKI_POSITION_________" << std::endl;
    std::cout << "pose.x: " << pose.x ;
    std::cout << "pose.y: " << pose.y ;
    std::cout << "theta : " << theta ;
    std::cout << "pose.z: " << pose.z << std::endl; 
#endif
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
	static int loop =0;
	static POSE MeansPose;
			
	pose.x = ob.pose.pose.pose.position.x*1000;
	pose.y = ob.pose.pose.pose.position.z*1000;
	pose.z = -1*ob.pose.pose.pose.position.y*1000;

	// 10 times data addeed
	MeansPose.x +=pose.x;
	MeansPose.y +=pose.y;
	MeansPose.z +=pose.z;


	// TransForm quaternion to revoltuion
	geometry_msgs::Quaternion quat;
	quat.x = ob.pose.pose.pose.orientation.x;
    quat.y = ob.pose.pose.pose.orientation.z;
    quat.z = -1*ob.pose.pose.pose.orientation.y;
    quat.w = ob.pose.pose.pose.orientation.w;

	pose.t = tf::getYaw(quat);
	MeansPose.t += pose.t;

	if(loop==PHASE1_LOOP){
		MeansPose.x /=loop;
		MeansPose.y /=loop;
		MeansPose.z /=loop;
		MeansPose.t /=loop;
		loop=0;
	}else{
		loop++;
	}
}

POSE Object::GetPosition()
{
	return pose;
}

void Object::SpeakPosition()
{
	std::cout << "____________Object Data____________" << std::endl;
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
	ROS_INFO(" JUST TEST STAGE_BEGIN");

	static int loop=0;

	//1st
	object_recognition_msgs::ObjectRecognitionGoal goal;
	ac->sendGoal(goal);

	bool finished_before_timeout = ac->waitForResult(ros::Duration(10.0));
	object_recognition_msgs::RecognizedObjectArray result;
	object_recognition_msgs::RecognizedObject *ob_;
	object_recognition_msgs::RecognizedObject MeansOb_[10];

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

	if(loop==PHASE1_LOOP){
		state = STAGE1;
		return SAFE;
	}else{
		loop++;
	}
}

///////////////////////////////////////////////////////////////////
///			P H A S E 1		"" A I M  T O  K O B U K I ""
///			1st 	Getting KOBUKI's position
///			2nd		Calcurate Diff Object position angle and KOBUKI position angle
///			3rd 	Move while this diff is near to ZERO
///////////////////////////////////////////////////////////////////
ROBO_CONDITION Robot::AimToObject()
{
	ROS_INFO(" JUST TEST STAGE1");
	// Debug Object
	int i=0;

	// INSTANCE
	POSE ob_pose;
	ob_pose = object[i].GetPosition();

	SpeakPosition();

	double AngleAim;

	if(ob_pose.x>0){
		AngleAim= atan( ob_pose.x / ob_pose.y)*-1;
	}else if(ob_pose.x<0){
		ob_pose.x *= -1;
		AngleAim= atan( ob_pose.x / ob_pose.y);
	}else{
		AngleAim=M_PI/2;
	}

	std::cout << "Angle Aim = " << AngleAim << std::endl;

	if(-0.15<AngleAim-theta && AngleAim-theta < 0.15){
		Stop();
		state = STAGE2;
		return SAFE;
	}else if(AngleAim-theta  > 0.15){
		std::cout <<"positive diff= " <<AngleAim-theta<<std::endl;
		Move(0,0,0.45);
		return MISS;
	}else if(AngleAim-theta  < -0.15){
		std::cout <<"negative diff= " <<AngleAim-theta<<std::endl;
		Move(0,0,-0.45);
		return MISS;
	}
}


///////////////////////////////////////////////////////////////////
///			P H A S E 2		"" Optimize Hand Position ""
///			1st		Calcurate distance from Diff Object position to Kobuki position
///			2nd		In order to be arms height equal to object height, calcurate y,x coodinates.
///////////////////////////////////////////////////////////////////
ROBO_CONDITION Robot::OptimizeHand(void)
{
	ROS_INFO(" JUST TEST STAGE");

	// preventer 
	static int loop=0;
	static bool CatchPoint=false;

	// Debug Object
	int i=0;

	const double ArmLength= 350;

	//		1st
	POSE ob_pose;							
	ob_pose = object[i].GetPosition();		//object data
	
	//calcurate distance from Object to Kobuki position	
	//double diff_y = ob_pose.y-pose.x;				// Distance 
	double diff_y = sqrt(ArmLength*ArmLength - ob_pose.z*ob_pose.z);
	double diff_z = ob_pose.z+300;						// Height
	
/*
	double ob_dis =0;
	
	std::cout << "____________OBJECT____DISTANCE____________" << std::endl;
	std::cout << "y_distance: " << diff_y << " z_distance: " << diff_z << std::endl;
	std::cout << "ob_dis =" << ob_dis << std::endl;
	loop++;
*/
	//		2nd
/*
	if(loop>20 && !CatchPoint){
		return DANGER;
	}
*/

	manip_and_recog::ArmPose srvHand;
	manip_and_recog::ArmPose srv;
/*
	if(ob_dis < 400){
		Move(-0.3,0,0);
	}else if(ob_dis > 630){
		Move(0.3,0,0);
	}else{
*/
	ROS_INFO(" ROBOT REACHED CATCHING POINT");
	//		3rd
	Stop();	

	srvHand.request.x = 000.00;
	srvHand.request.y = 000.00;
	srvHand.request.motion = "open";

	if(Armclient.call(srvHand)){
		ROS_INFO("______HAND_OPENED______");
	}else{
		ROS_INFO(" HAND OPENED FALSE");
		return DANGER;
	}

	//		4th
	srv.request.x = diff_y;
	srv.request.y = diff_z;
	srv.request.motion = "catch";
	ROS_INFO("Send Position Data");

	if(Armclient.call(srv)){
		ROS_INFO(" Move SUCCESS");
		state = STAGE3;
		return SAFE;
	}else{
		ROS_INFO(" Move FALSE");
		return DANGER;
	}
//	}
}
///////////////////////////////////////////////////////////////////
///			P H A S E 3		"" Move to catching point ""
///			1st		Calcurate distance from Diff Object position to Kobuki position
///			2nd		if this distance is far -> KOBUKI move to Object
///////////////////////////////////////////////////////////////////
ROBO_CONDITION Robot::MoveToObject(void)
{
	ROS_INFO(" JUST TEST STAGE2");

	// preventer 
	static int loop=0;
	static bool CatchPoint=false;

	// Debug Object
	int i=0;

	//		1st
	POSE ob_pose;							
	ob_pose = object[i].GetPosition();		//object data
		
	double diff_y = ob_pose.y-pose.x;		//calcurate distance from Object to Kobuki position
	double diff_z = ob_pose.z;
	
	double ob_dis =0;
	ob_dis = sqrt(diff_y*diff_y + diff_z*diff_z);		

	std::cout << "____________OBJECT____DISTANCE____________" << std::endl;
	std::cout << "y_distance: " << diff_y << " z_distance: " << diff_z << std::endl;
	std::cout << "ob_dis =" << ob_dis << std::endl;
	loop++;

	//		2nd
/*
	if(loop>20 && !CatchPoint){
		return DANGER;
	}
*/

	manip_and_recog::ArmPose srvHand;
	manip_and_recog::ArmPose srv;

	if(ob_dis < 400){
		Move(-0.3,0,0);
	}else if(ob_dis > 630){
		Move(0.3,0,0);
	}else{
		ROS_INFO(" ROBOT REACHED CATCHING POINT");
		//		3rd
		Stop();	

		srvHand.request.x = 000.00;
		srvHand.request.y = 000.00;
		srvHand.request.motion = "open";

		if(Armclient.call(srvHand)){
			ROS_INFO("______HAND_OPENED______");
		}else{
			ROS_INFO(" HAND OPENED FALSE");
			return DANGER;
		}

		//		4th
		srv.request.x = diff_y;
		srv.request.y = diff_z;
		srv.request.motion = "catch";
		ROS_INFO("Send Position Data");

		if(Armclient.call(srv)){
			ROS_INFO(" Move SUCCESS");
			state = STAGE4;
			return SAFE;
		}else{
			ROS_INFO(" Move FALSE");
			return DANGER;
		}
	}
}


///////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
	ROS_INFO("MANIP_AND_RECOGNITION   S T A R T");
	ros::init(argc, argv, "manip_and_recog");

	Robot robot;
	
	robot.init();
	

	ros::Rate loop_rate(40);   
		
	int loop;

	PHASE_STATE STAGE_NUMBER=STAGE_BEGIN;				//		PHASE NUMBER
	robot.setState( STAGE_BEGIN );

	ROBO_CONDITION result;				
	
	robot.HeadUp();

	while(ros::ok()){
		if(CallMini){
			ROS_INFO(" Called Mini");
		

			switch( robot.getState() ){
				std::cout<<robot.getState()<<std::endl;
		    	case STAGE_BEGIN:	result=robot.GetDataObject();		break;
		    	case STAGE1:		result=robot.AimToObject();			break;	
		    	case STAGE2:		result=robot.OptimizeHand();		break;
		    	case STAGE3:		result=robot.MoveToObject();		break;
		    	
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
		    	loop++;
		    	ROS_INFO("PHASE_No.%d is DANGER\n",robot.getState());

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
	    }else{
	    	ROS_INFO(" Please say Mini");
	    }
	    loop_rate.sleep();
		ros::spinOnce();
	}
	
    ROS_INFO("FINISHED");

	return 0;
}
