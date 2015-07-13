	bool flag_GetData = true;

	for(int i =0; i<ObjectsNum; i++){
		pose = objects[i].pose.pose.pose.position;

		if( !pose.x && !pose.y && !pose.z && i==0){
			flag_GetData=false;
			return false;
		}else{
			Objects[i].x=pose.x;
			Objects[i].y=pose.y;
			Objects[i].z=pose.z;
			Objects[i].t=0;	
		}
	}

///////////////////////////////////////////////////////////////////
///
///		STAGE 1 Aim to Objects
///		1. Data has position or doesn't
///		2. transform data_position to world position
///		3. revolve KOBUKI in order to face with object
///
///		bool STAGE_FAST(int num)-----int num -> Object Counter (Max 5)
///////////////////////////////////////////////////////////////////
POSE ob[10];

bool STAGE_FAST(int num)
{
	if( !GetDataObject() )
	{
		ROS_INFO("NOT data form ActionLib\n");
		return false;
	}
	
	for(int i =0; i<ObjectsNum; i++){
		ob[i].x = Objects[i].z;
		ob[i].y = Objects[i].x;
		ob[i].z = -1*Objects[i].y;
		ob[i].t = tan(ob[i].y/ob[i].x);
	}

#ifdef DEBUG
	for(int i =0; i<ObjectsNum; i++){
		std::cout << "OBJECT POTISION X=" << ob[i].x << std::endl;
		std::cout << "OBJECT POTISION Y=" << ob[i].y << std::endl;
		std::cout << "OBJECT POTISION Z=" << ob[i].z << std::endl;
		std::cout << "OBJECT POTISION T=" << ob[i].t << std::endl;
	}
#endif

	while( ob[num].t < -5*180/M_PI && ob[num].t > 5*180/M_PI){
		if(ob[num].t >0.0){
			Move(0.0, 0.0, 4.0);
		}else{
			Move(0.0, 0.0, -4.0);
		}
	}

	ROS_INFO("ROBOT has aimed object now\n");

	while(ob[num].x > 1000){
		Move(3.0, 0.0, 0.0);
	}

	ROS_INFO("ROBOT has moved to catching point now\n");
	return true;
}

///////////////////////////////////////////////////////////////////
///
///		STAGE 2 SET UP ARM POSITION
///		1. set up request file
///		2. send this request file
///		3. result back analyse 
///		
///////////////////////////////////////////////////////////////////

    
bool STAGE_SECOND(int num){
	ros::NodeHandle node;
	
	manip_and_recog::ArmPose srv;

	srv.request.x =  ob[num].x;
	srv.request.y =  ob[num].y;
	srv.request.motion =  "catch";

	if(client.call(srv)){
		if(srv.response.result ==1){
			ROS_INFO("PROGRMA send request to Arm_POSE completly");
		}else{
			ROS_INFO("PROGRMA send request to Arm_POSE but errors occured");
		}
	}else{
		ROS_ERROR("Failed to call service arm_pose");
	}
}

///////////////////////////////////////////////////////////////////
///
///		STAGE 3 putting up
///		1. setup service and timer counter
///		2. arm hand graping objects put up 6cm
///		3. do srv.result
///		
///////////////////////////////////////////////////////////////////

bool STAGE_THIRD(int num){

	ros::ServiceClient client = node.serviceClient<manip_and_recog::ArmPose>("arm_pose");

	manip_and_recog::ArmPose srv;

	srv.request.x =  ob[num].x;
	srv.request.y =  ob[num].y+60;
	srv.request.motion =  "catch";

	if(client.call(srv)){
		if(srv.response.result ==1){
			ROS_INFO("PROGRMA send request to Arm_POSE completly");
		}else{
			ROS_INFO("PROGRMA send request to Arm_POSE but errors occured");
		}
	}else{
		ROS_ERROR("Failed to call service arm_pose");
	}
}

///////////////////////////////////////////////////////////////////
///
///		STAGE 4 putting up
///		
///		caution: there is grasping objects in middle shelf
///
///		1. arm put down 
///		2. count 10second in grasping objects 
///		3. remain arm positon but hand release
///////////////////////////////////////////////////////////////////
/*
bool flag_count=false;
bool flag_10s=false;
bool flag_putdown=false;

void callbackTime(const ros::TimerEvents&)
{
	static int count=0;
	if(flag_count)
	{
		count++
		if(count==2){
			flag_putdown=true;
		}
	}
}

bool STAGE_FOUTH(int num){
	ros::NodeHandle node;
	ros::ServiceClient client = node.serviceClient<manip_and_recog::ArmPose>("arm_pose");
	ros::ServiceClient client2 = node.serviceClient<manip_and_recog::Servo>("arm_servo");
	ros::Timer timer = node.createTimer(ros::Duration(1.0), callbackTime);

	manip_and_recog::ArmPose srv;
	manip_and_recog::Servo srv2;

	if(!flag_putdown){
		srv.request.x =  ob[num].x;
		srv.request.y =  ob[num].y;
		srv.request.motion =  "catch";

		if(client.call(srv)){
			if(srv.response.result ==1){
				ROS_INFO("PROGRMA send request to Arm_POSE completly");
				flag_count=true;
			}else{
				ROS_INFO("PROGRMA send request to Arm_POSE but errors occured");
			}
		}else{
			ROS_ERROR("Failed to call service arm_pose");
		}
	}else{
		srv2.num=9;
		srv2.angle=200;
		srv2.motion="catch";

		if(client2.call(srv2)){
			if(srv2.response.result ==1){
				ROS_INFO("PROGRMA send request to Arm_POSE completly");
			}else{
				ROS_INFO("PROGRMA send request to Arm_POSE but errors occured");
			}
		}else{
			ROS_ERROR("Failed to call service arm_pose");
		}
	}
}
*/
///////////////////////////////////////////////////////////////////
///
///		STAGE 6 putting up and down
///		1. after STAGE 5, arm is free
///		2. kobuki back 500cm
///     3. reached 500cm back finished 
///////////////////////////////////////////////////////////////////
/*
bool STAGE_SIXTH(int num){
	



}*/


