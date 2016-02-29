#include "ros/ros.h"
#include "dynamixel_controllers/SetComplianceMargin.h"
#include "dynamixel_controllers/SetCompliancePunch.h"
#include "dynamixel_controllers/SetComplianceSlope.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/TorqueEnable.h"
#include "dynamixel_controllers/SetTorqueLimit.h"
#include "dynamixel_msgs/JointState.h"
#include "bender_srvs/Margin.h"
#include "bender_srvs/Punch.h"
#include "bender_srvs/Slope.h"
#include "bender_srvs/Speed.h"
#include "bender_srvs/TorqueEnable.h"
#include "bender_srvs/TorqueLimit.h"
#include "bender_srvs/State.h"
#include "bender_srvs/Onoff.h"
#include "std_msgs/String.h"
#include <string>

ros::ServiceClient call_margin[8];
ros::ServiceClient call_punch[8];
ros::ServiceClient call_slope[8];
ros::ServiceClient call_speed[8];
ros::ServiceClient call_torque[8];
ros::ServiceClient call_torque_limit[8];
ros::ServiceClient call_state[8];

dynamixel_controllers::SetComplianceMargin ma;
dynamixel_controllers::SetCompliancePunch pu;
dynamixel_controllers::SetComplianceSlope slo;
dynamixel_controllers::SetSpeed spe;
dynamixel_controllers::TorqueEnable tor;
dynamixel_controllers::SetTorqueLimit tlim;

bool sameMargin(bender_srvs::Margin::Request &req,
		bender_srvs::Margin::Response &res);

bool samePunch(bender_srvs::Punch::Request &req,
		bender_srvs::Punch::Response &res);

bool sameSlope(bender_srvs::Slope::Request &req,
		bender_srvs::Slope::Response &res);

bool sameSpeed(bender_srvs::Speed::Request &req,
		bender_srvs::Speed::Response &res);

bool sameTorque(bender_srvs::TorqueEnable::Request &req,
		bender_srvs::TorqueEnable::Response &res);

bool sameTorqueLimit(bender_srvs::TorqueLimit::Request &req,
		bender_srvs::TorqueLimit::Response &res);

bool turnOn(bender_srvs::Onoff::Request &req,
		bender_srvs::Onoff::Response &res);

int setNumero(std::basic_string<char> motor);


bool getArmState(bender_srvs::State::Request &req,
		bender_srvs::State::Response &res);


bool prueba(bender_srvs::TorqueEnable::Request &req, bender_srvs::TorqueEnable::Response &res);

void joints_pos(dynamixel_msgs::JointState codo);

double left_arm_state [8], left_arm_load [8];

double left_pos[8],left_load[8];
bool left_move[8];

int main(int argc, char **argv)
{
	ros::init(argc,argv,"interface_plan_arm");
	ros::NodeHandle n;

	ros::Subscriber hom1 = n.subscribe("hombro_1_controller/state", 1000, joints_pos);
	ros::Subscriber hom2 = n.subscribe("hombro_2_controller/state", 1000, joints_pos);
	ros::Subscriber hom3 = n.subscribe("hombro_3_controller/state", 1000, joints_pos);
	ros::Subscriber cod1 = n.subscribe("codo_1_controller/state", 1000, joints_pos);
	ros::Subscriber cod2 = n.subscribe("codo_2_controller/state", 1000, joints_pos);
	ros::Subscriber mun = n.subscribe("muneca_controller/state", 1000, joints_pos);
	ros::Subscriber ded1 = n.subscribe("dedo_1_controller/state", 1000, joints_pos);
	ros::Subscriber ded2 = n.subscribe("dedo_2_controller/state", 1000, joints_pos);

	ros::ServiceServer marg = n.advertiseService("left_arm_joints/set_compliance_margin", sameMargin);
	ros::ServiceServer slop = n.advertiseService("left_arm_joints/set_compliance_slope", sameSlope);
	ros::ServiceServer punc = n.advertiseService("left_arm_joints/set_compliance_punch", samePunch);
	ros::ServiceServer spee = n.advertiseService("left_arm_joints/set_speed", sameSpeed);
	ros::ServiceServer torq = n.advertiseService("left_arm_joints/torque_enable", sameTorque);
	ros::ServiceServer torqlim = n.advertiseService("left_arm_joints/torque_limit", sameTorqueLimit);
	ros::ServiceServer stat = n.advertiseService("left_arm_joints/states", getArmState);
	ros::ServiceServer onoff = n.advertiseService("left_arm_joints/onoff", turnOn);

	call_margin[0] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("hombro_1_controller/set_compliance_margin");
	call_margin[1] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("hombro_2_controller/set_compliance_margin");
	call_margin[2] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("hombro_3_controller/set_compliance_margin");
	call_margin[3] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("codo_1_controller/set_compliance_margin");
	call_margin[4] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("codo_2_controller/set_compliance_margin");
	call_margin[5] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("muneca_controller/set_compliance_margin");
	call_margin[6] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("dedo_1_controller/set_compliance_margin");
	call_margin[7] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("dedo_2_controller/set_compliance_margin");

	call_slope[0] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("hombro_1_controller/set_compliance_slope");
	call_slope[1] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("hombro_2_controller/set_compliance_slope");
	call_slope[2] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("hombro_3_controller/set_compliance_slope");
	call_slope[3] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("codo_1_controller/set_compliance_slope");
	call_slope[4] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("codo_2_controller/set_compliance_slope");
	call_slope[5] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("muneca_controller/set_compliance_slope");
	call_slope[6] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("dedo_1_controller/set_compliance_slope");
	call_slope[7] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("dedo_2_controller/set_compliance_slope");

	call_punch[0] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("hombro_1_controller/set_compliance_punch");
	call_punch[1] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("hombro_2_controller/set_compliance_punch");
	call_punch[2] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("hombro_3_controller/set_compliance_punch");
	call_punch[3] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("codo_1_controller/set_compliance_punch");
	call_punch[4] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("codo_2_controller/set_compliance_punch");
	call_punch[5] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("muneca_controller/set_compliance_punch");
	call_punch[6] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("dedo_1_controller/set_compliance_punch");
	call_punch[7] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("dedo_2_controller/set_compliance_punch");

	call_speed[0] = n.serviceClient<dynamixel_controllers::SetSpeed>("hombro_1_controller/set_speed");
	call_speed[1] = n.serviceClient<dynamixel_controllers::SetSpeed>("hombro_2_controller/set_speed");
	call_speed[2] = n.serviceClient<dynamixel_controllers::SetSpeed>("hombro_3_controller/set_speed");
	call_speed[3] = n.serviceClient<dynamixel_controllers::SetSpeed>("codo_1_controller/set_speed");
	call_speed[4] = n.serviceClient<dynamixel_controllers::SetSpeed>("codo_2_controller/set_speed");
	call_speed[5] = n.serviceClient<dynamixel_controllers::SetSpeed>("muneca_controller/set_speed");
	call_speed[6] = n.serviceClient<dynamixel_controllers::SetSpeed>("dedo_1_controller/set_speed");
	call_speed[7] = n.serviceClient<dynamixel_controllers::SetSpeed>("dedo_2_controller/set_speed");

	call_torque[0] = n.serviceClient<dynamixel_controllers::TorqueEnable>("hombro_1_controller/torque_enable");
	call_torque[1] = n.serviceClient<dynamixel_controllers::TorqueEnable>("hombro_2_controller/torque_enable");
	call_torque[2] = n.serviceClient<dynamixel_controllers::TorqueEnable>("hombro_3_controller/torque_enable");
	call_torque[3] = n.serviceClient<dynamixel_controllers::TorqueEnable>("codo_1_controller/torque_enable");
	call_torque[4] = n.serviceClient<dynamixel_controllers::TorqueEnable>("codo_2_controller/torque_enable");
	call_torque[5] = n.serviceClient<dynamixel_controllers::TorqueEnable>("muneca_controller/torque_enable");
	call_torque[6] = n.serviceClient<dynamixel_controllers::TorqueEnable>("dedo_1_controller/torque_enable");
	call_torque[7] = n.serviceClient<dynamixel_controllers::TorqueEnable>("dedo_2_controller/torque_enable");

	call_torque_limit[0] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("hombro_1_controller/set_torque_limit");
	call_torque_limit[1] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("hombro_2_controller/set_torque_limit");
	call_torque_limit[2] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("hombro_3_controller/set_torque_limit");
	call_torque_limit[3] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("codo_1_controller/set_torque_limit");
	call_torque_limit[4] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("codo_2_controller/set_torque_limit");
	call_torque_limit[5] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("muneca_controller/set_torque_limit");
	call_torque_limit[6] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("dedo_1_controller/set_torque_limit");
	call_torque_limit[7] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("dedo_2_controller/set_torque_limit");

	ros::spin();

	return 0;
}

bool prueba(bender_srvs::TorqueEnable::Request &req, bender_srvs::TorqueEnable::Response &res)
{

	int motor=setNumero(req.motor_name.c_str());
	tor.request.torque_enable=req.torque_enable;
	call_torque[5].call(tor);
	ROS_INFO("%s %d %s",req.motor_name.c_str(), motor, (tor.request.torque_enable)?"true":"false");

	return true;
}

bool sameMargin(bender_srvs::Margin::Request &req,
		bender_srvs::Margin::Response &res)
{	
	int motor = setNumero(req.motor_name.c_str());
	switch(motor)
	{
		case 1:
			{
				ma.request.margin=req.margin;
				call_margin[0].call(ma);
				break;
			}
		case 2:
			{
				ma.request.margin=req.margin;
				call_margin[1].call(ma);
				break;
			}
		case 3:
			{
				ma.request.margin=req.margin;
				call_margin[2].call(ma);
				break;
			}
		case 4:
			{
				ma.request.margin=req.margin;
				call_margin[3].call(ma);
				break;
			}
		case 5:
			{
				ma.request.margin=req.margin;
				call_margin[4].call(ma);
				break;
			}
		case 6:
			{
				ma.request.margin=req.margin;
				call_margin[5].call(ma);
				break;
			}
		case 7:
			{
				ma.request.margin=req.margin;
				call_margin[6].call(ma);
				break;
			}
		case 8:
			{
				ma.request.margin=req.margin;
				call_margin[7].call(ma);
				break;
			}
		default:
		{
			ROS_ERROR("failed to call /set_compliance_margin");
			break;
		}

	}

	return true;
}

bool samePunch(bender_srvs::Punch::Request &req,
		bender_srvs::Punch::Response &res)
{
	int motor=setNumero(req.motor_name.c_str());
	switch (motor)
		{
			case 1:
				{
					pu.request.punch=req.punch;
					call_punch[0].call(pu);
					break;
				}
			case 2:
				{
					pu.request.punch=req.punch;
					call_punch[1].call(pu);
					break;
				}
			case 3:
				{
					pu.request.punch=req.punch;
					call_punch[2].call(pu);
					break;
				}
			case 4:
				{
					pu.request.punch=req.punch;
					call_punch[3].call(pu);
					break;
				}
			case 5:
				{
					pu.request.punch=req.punch;
					call_punch[4].call(pu);
					break;
				}
			case 6:
				{
					pu.request.punch=req.punch;
					call_punch[5].call(pu);
					break;
				}
			case 7:
				{
					pu.request.punch=req.punch;
					call_punch[6].call(pu);
					break;
				}
			case 8:
				{
					pu.request.punch=req.punch;
					call_punch[7].call(pu);
					break;
				}
			default:
				{
					ROS_ERROR("failed to call /set_compliance_punch");
					break;
				}
		}
	return true;
}

bool sameSlope(bender_srvs::Slope::Request &req,
		bender_srvs::Slope::Response &res)
{
		int motor=setNumero(req.motor_name.c_str());
		switch (motor)
			{
				case 1:
					{
						slo.request.slope=req.slope;
						call_slope[0].call(slo);
						break;
					}
				case 2:
					{
						slo.request.slope=req.slope;
						call_slope[1].call(slo);
						break;
					}
				case 3:
					{
						slo.request.slope=req.slope;
						call_slope[2].call(slo);
						break;
					}
				case 4:
					{
						slo.request.slope=req.slope;
						call_slope[3].call(slo);
						break;
					}
				case 5:
					{
						slo.request.slope=req.slope;
						call_slope[4].call(slo);
						break;
					}
				case 6:
					{
						slo.request.slope=req.slope;
						call_slope[5].call(slo);
						break;
					}
				case 7:
					{
						slo.request.slope=req.slope;
						call_slope[6].call(slo);
						break;
					}
				case 8:
					{
						slo.request.slope=req.slope;
						call_slope[7].call(slo);
						break;
					}
				default:
					{
						ROS_ERROR("failed to call /set_compliance_slope");
						break;
					}
			}
		return true;
}

bool sameSpeed(bender_srvs::Speed::Request &req,
		bender_srvs::Speed::Response &res)
{
	int motor=setNumero(req.motor_name.c_str());
			switch (motor)
				{
					case 1:
						{
							spe.request.speed=req.speed;
							call_speed[0].call(spe);
							break;
						}
					case 2:
						{
							spe.request.speed=req.speed;
							call_speed[1].call(spe);
							break;
						}
					case 3:
						{
							spe.request.speed=req.speed;
							call_speed[2].call(spe);
							break;
						}
					case 4:
						{
							spe.request.speed=req.speed;
							call_speed[3].call(spe);
							break;
						}
					case 5:
						{
							spe.request.speed=req.speed;
							call_speed[4].call(spe);
							break;
						}
					case 6:
						{
							spe.request.speed=req.speed;
							call_speed[5].call(spe);
							break;
						}
					case 7:
						{
							spe.request.speed=req.speed;
							call_speed[6].call(spe);
							break;
						}
					case 8:
						{
							spe.request.speed=req.speed;
							call_speed[7].call(spe);
							break;
						}
					default:
						{
							ROS_ERROR("failed to call /set_speed");
							break;
						}
				}
			return true;
}

bool sameTorque(bender_srvs::TorqueEnable::Request &req,
		bender_srvs::TorqueEnable::Response &res)
{
	int motor=setNumero(req.motor_name.c_str());
	switch (motor)
		{
			case 1:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[0].call(tor);
					break;
				}
			case 2:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[1].call(tor);
					break;
				}
			case 3:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[2].call(tor);
					break;
				}
			case 4:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[3].call(tor);
					break;
				}
			case 5:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[4].call(tor);
					break;
				}
			case 6:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[5].call(tor);
					break;
				}
			case 7:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[6].call(tor);
					break;
				}
			case 8:
				{
					tor.request.torque_enable=req.torque_enable;
					call_torque[7].call(tor);
					break;
				}
			default:
				{
					ROS_ERROR("failed to call /torque_enable");
					break;
				}
			}
			return true;
}

bool sameTorqueLimit(bender_srvs::TorqueLimit::Request &req,
		bender_srvs::TorqueLimit::Response &res)
{
	int motor=setNumero(req.motor_name.c_str());
	switch (motor)
		{
			case 1:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[0].call(tlim);
					break;
				}
			case 2:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[1].call(tlim);
					break;
				}
			case 3:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[2].call(tlim);
					break;
				}
			case 4:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[3].call(tlim);
					break;
				}
			case 5:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[4].call(tlim);
					break;
				}
			case 6:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[5].call(tlim);
					break;
				}
			case 7:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[6].call(tlim);
					break;
				}
			case 8:
				{
					tlim.request.torque_limit=req.torque_limit;
					call_torque_limit[7].call(tlim);
					break;
				}
			default:
				{
					ROS_ERROR("failed to call /torque_limit");
					break;
				}
			}
			return true;
}

int setNumero(std::basic_string<char> motor)
{
	int a=0;
	if (motor.compare("hombro_1")==0)
		{
			a=1;
		}
	else if (motor.compare("hombro_2")==0)
		{
			a=2;
		}
	else if (motor.compare("hombro_3")==0)
		{
			a=3;
		}
	else if (motor.compare("codo_1")==0)
		{
			a=4;
		}
	else if (motor.compare("codo_2")==0)
		{
			a=5;
		}
	else if (motor.compare("muneca")==0)
		{
			a=6;
		}
	else if (motor.compare("dedo_1")==0)
		{
			a=7;
		}
	else if (motor.compare("dedo_2")==0)
		{
			a=8;
		}
	return a;

}

bool getArmState(bender_srvs::State::Request &req,
		bender_srvs::State::Response &res)
{
	req.select.resize(8);
	res.state.resize(8);
	res.load.resize(8);
	res.is_moving.resize(8);

	for(int i=0;i<8;i++)
	{
		if(req.select[i])
		{
			res.state[i]=left_pos[i];
			res.load[i]=left_load[i];
			res.is_moving[i]=left_move[i];
		}
	}
	return true;
}

void joints_pos(dynamixel_msgs::JointState joint)
{

	if (joint.motor_ids[0]==1)
    {
        left_pos[0] = joint.current_pos;
        left_load[0] = joint.load;
        left_move[0] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==2)
    {
    	left_pos[1] = joint.current_pos;
    	left_load[1] = joint.load;
    	left_move[1] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==3)
    {
    	left_pos[2] = joint.current_pos;
    	left_load[2] = joint.load;
    	left_move[2] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==4)
    {
    	left_pos[3] = joint.current_pos;
    	left_load[3] = joint.load;
    	left_move[3] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==5)
    {
      	left_pos[4] = joint.current_pos;
       	left_load[4] = joint.load;
       	left_move[4] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==6)
    {
        left_pos[5] = joint.current_pos;
       	left_load[5] = joint.load;
       	left_move[5] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==7)
    {
       	left_pos[6] = joint.current_pos;
       	left_load[6] = joint.load;
       	left_move[6] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==8)
    {
       	left_pos[7] = joint.current_pos;
       	left_load[7] = joint.load;
       	left_move[7] = joint.is_moving;
    }

    return;

}

bool turnOn(bender_srvs::Onoff::Request &req,
		bender_srvs::Onoff::Response &res)
{
	if (req.select == true)
	{
		tor.request.torque_enable=true;
		for(int i=0; i<8 ; i++)
		{
			call_torque[i].call(tor);
		}
	}
	else if (req.select == false)
	{
		tor.request.torque_enable=false;
		for(int i=0; i<8 ; i++)
		{
			call_torque[i].call(tor);
		}
	}
	return true;
}
