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
#include "bender_srvs/States.h"
#include "bender_srvs/Onoff.h"
#include "bender_srvs/TorqueLimit.h"
#include "std_msgs/String.h"
#include <string>

ros::ServiceClient call_margin[6];
ros::ServiceClient call_punch[6];
ros::ServiceClient call_slope[6];
ros::ServiceClient call_speed[6];
ros::ServiceClient call_torque[6];
ros::ServiceClient call_state[6];
ros::ServiceClient call_torquelimit[6];

dynamixel_controllers::SetComplianceMargin ma;
dynamixel_controllers::SetCompliancePunch pu;
dynamixel_controllers::SetComplianceSlope slo;
dynamixel_controllers::SetSpeed spe;
dynamixel_controllers::TorqueEnable tor;
dynamixel_controllers::SetTorqueLimit torli;

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

bool sameTorqueLimit(bender_srvs::TorqueLimit::Request &req , bender_srvs::TorqueLimit::Response &res);

bool turnOn(bender_srvs::Onoff::Request &req,
		bender_srvs::Onoff::Response &res);

int setNumero(std::basic_string<char> motor);


bool getArmState(bender_srvs::States::Request &req,
		bender_srvs::States::Response &res);


bool prueba(bender_srvs::TorqueEnable::Request &req, bender_srvs::TorqueEnable::Response &res);

void joints_pos(dynamixel_msgs::JointState codo);

double bottom_arm_state [6], bottom_arm_load [6];

double bottom_pos[6],bottom_load[6];
bool bottom_move[6];

int main(int argc, char **argv)
{
	ros::init(argc,argv,"interface_plan_arm");
	ros::NodeHandle n;

	ros::Subscriber m1 = n.subscribe("m_1_controller/state", 1000, joints_pos);
	ros::Subscriber m2 = n.subscribe("m_2_controller/state", 1000, joints_pos);
	ros::Subscriber m3 = n.subscribe("m_3_controller/state", 1000, joints_pos);
	ros::Subscriber m4 = n.subscribe("m_4_controller/state", 1000, joints_pos);
	ros::Subscriber m5 = n.subscribe("m_5_controller/state", 1000, joints_pos);
	ros::Subscriber m6 = n.subscribe("m_6_controller/state", 1000, joints_pos);

	ros::ServiceServer marg = n.advertiseService("bottom_arm_joints/set_compliance_margin", sameMargin);
	ros::ServiceServer slop = n.advertiseService("bottom_arm_joints/set_compliance_slope", sameSlope);
	ros::ServiceServer punc = n.advertiseService("bottom_arm_joints/set_compliance_punch", samePunch);
	ros::ServiceServer spee = n.advertiseService("bottom_arm_joints/set_speed", sameSpeed);
	ros::ServiceServer torq = n.advertiseService("bottom_arm_joints/torque_enable", sameTorque);
	ros::ServiceServer stat = n.advertiseService("bottom_arm_joints/states", getArmState);
	ros::ServiceServer onoff = n.advertiseService("bottom_arm_joints/onoff", turnOn);
	ros::ServiceServer torql = n.advertiseService("bottom_arm_joints/set_torque_limit", sameTorqueLimit);

	call_margin[0] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("m_1_controller/set_compliance_margin");
	call_margin[1] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("m_2_controller/set_compliance_margin");
	call_margin[2] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("m_3_controller/set_compliance_margin");
	call_margin[3] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("m_4_controller/set_compliance_margin");
	call_margin[4] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("m_5_controller/set_compliance_margin");
	call_margin[5] = n.serviceClient<dynamixel_controllers::SetComplianceMargin>("m_6_controller/set_compliance_margin");

	call_slope[0] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("m_1_controller/set_compliance_slope");
	call_slope[1] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("m_2_controller/set_compliance_slope");
	call_slope[2] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("m_3_controller/set_compliance_slope");
	call_slope[3] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("m_4_controller/set_compliance_slope");
	call_slope[4] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("m_5_controller/set_compliance_slope");
	call_slope[5] = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("m_6_controller/set_compliance_slope");

	call_punch[0] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("m_1_controller/set_compliance_punch");
	call_punch[1] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("m_2_controller/set_compliance_punch");
	call_punch[2] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("m_3_controller/set_compliance_punch");
	call_punch[3] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("m_4_controller/set_compliance_punch");
	call_punch[4] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("m_5_controller/set_compliance_punch");
	call_punch[5] = n.serviceClient<dynamixel_controllers::SetCompliancePunch>("m_6_controller/set_compliance_punch");

	call_speed[0] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_1_controller/set_speed");
	call_speed[1] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_2_controller/set_speed");
	call_speed[2] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_3_controller/set_speed");
	call_speed[3] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_4_controller/set_speed");
	call_speed[4] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_5_controller/set_speed");
	call_speed[5] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_6_controller/set_speed");

	call_torque[0] = n.serviceClient<dynamixel_controllers::TorqueEnable>("m_1_controller/torque_enable");
	call_torque[1] = n.serviceClient<dynamixel_controllers::TorqueEnable>("m_2_controller/torque_enable");
	call_torque[2] = n.serviceClient<dynamixel_controllers::TorqueEnable>("m_3_controller/torque_enable");
	call_torque[3] = n.serviceClient<dynamixel_controllers::TorqueEnable>("m_4_controller/torque_enable");
	call_torque[4] = n.serviceClient<dynamixel_controllers::TorqueEnable>("m_5_controller/torque_enable");
	call_torque[5] = n.serviceClient<dynamixel_controllers::TorqueEnable>("m_6_controller/torque_enable");

	call_torquelimit[0] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("m_1_controller/set_torque_limit");
	call_torquelimit[1] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("m_2_controller/set_torque_limit");
	call_torquelimit[2] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("m_3_controller/set_torque_limit");
	call_torquelimit[3] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("m_4_controller/set_torque_limit");
	call_torquelimit[4] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("m_5_controller/set_torque_limit");
	call_torquelimit[5] = n.serviceClient<dynamixel_controllers::SetTorqueLimit>("m_6_controller/set_torque_limit");

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
		default:
		{
			ROS_ERROR("DSADSAD");
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
			default:
				{
					ROS_ERROR("DSADSAD");
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
				default:
					{
						ROS_ERROR("DSADSAD");
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
					default:
						{
							ROS_ERROR("DSADSAD");
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
			default:
				{
					ROS_ERROR("DSADSAD");
					break;
				}
			}
			return true;
}

bool sameTorqueLimit(bender_srvs::TorqueLimit::Request &req , bender_srvs::TorqueLimit::Response &res)
{
	int motor=setNumero(req.motor_name.c_str());
	switch (motor)
		{
			case 1:
				{
					torli.request.torque_limit=req.torque_limit;
					call_torquelimit[0].call(torli);
					break;
				}
			case 2:
				{
					torli.request.torque_limit=req.torque_limit;
					call_torquelimit[1].call(torli);
					break;
				}
			case 3:
				{
					torli.request.torque_limit=req.torque_limit;
					call_torquelimit[2].call(torli);
					break;
				}
			case 4:
				{
					torli.request.torque_limit=req.torque_limit;
					call_torquelimit[3].call(torli);
					break;
				}
			case 5:
				{
					torli.request.torque_limit=req.torque_limit;
					call_torquelimit[4].call(torli);
					break;
				}
			case 6:
				{
					torli.request.torque_limit=req.torque_limit;
					call_torquelimit[5].call(torli);
					break;
				}
			default:
				{
					ROS_ERROR("DSADSAD");
					break;
				}
			}
		return true;
}


int setNumero(std::basic_string<char> motor)
{
	int a=0;
	if (motor.compare("m_1")==0)
		{
			a=1;
		}
	else if (motor.compare("m_2")==0)
		{
			a=2;
		}
	else if (motor.compare("m_3")==0)
		{
			a=3;
		}
	else if (motor.compare("m_4")==0)
		{
			a=4;
		}
	else if (motor.compare("m_5")==0)
		{
			a=5;
		}
	else if (motor.compare("m_6")==0)
		{
			a=6;
		}
	return a;

}

bool getArmState(bender_srvs::States::Request &req,
		bender_srvs::States::Response &res)
{
	req.select.resize(6);
	res.state.resize(6);
	res.load.resize(6);
	res.is_moving.resize(6);

	for(int i=0;i<6;i++)
	{
		if(req.select[i])
		{
			res.state[i]=bottom_pos[i];
			res.load[i]=bottom_load[i];
			res.is_moving[i]=bottom_move[i];
		}
	}
	return true;
}

void joints_pos(dynamixel_msgs::JointState joint)
{

	if (joint.motor_ids[0]==21)
    {
        bottom_pos[0] = joint.current_pos;
        bottom_load[0] = joint.load;
        bottom_move[0] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==22)
    {
    	bottom_pos[1] = joint.current_pos;
    	bottom_load[1] = joint.load;
    	bottom_move[1] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==23)
    {
    	bottom_pos[2] = joint.current_pos;
    	bottom_load[2] = joint.load;
    	bottom_move[2] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==24)
    {
    	bottom_pos[3] = joint.current_pos;
    	bottom_load[3] = joint.load;
    	bottom_move[3] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==25)
    {
      	bottom_pos[4] = joint.current_pos;
       	bottom_load[4] = joint.load;
       	bottom_move[4] = joint.is_moving;
    }
    else if (joint.motor_ids[0]==26)
    {
        bottom_pos[5] = joint.current_pos;
       	bottom_load[5] = joint.load;
       	bottom_move[5] = joint.is_moving;
    }

    return;

}

bool turnOn(bender_srvs::Onoff::Request &req,
		bender_srvs::Onoff::Response &res)
{
	if (req.select == true)
	{
		tor.request.torque_enable=true;
		for(int i=0; i<6 ; i++)
		{
			call_torque[i].call(tor);
		}
	}
	else if (req.select == false)
	{
		tor.request.torque_enable=false;
		for(int i=0; i<6 ; i++)
		{
			call_torque[i].call(tor);
		}
	}
	return true;
}
