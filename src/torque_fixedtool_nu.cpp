#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "Vector3.h"
#include "Matrix33.h"
#include "math.h"

#define loopRate 250

using namespace matrix;
using namespace std;

geometry_msgs::TwistStamped mes_vel;//measured angular velocity
geometry_msgs::PoseStamped mes_pos;//measured position
geometry_msgs::TwistStamped cmd_w;//command angular velocity
mavros_msgs::State current_state;
std_msgs::Float64 cmd_thr;

//desired motion
Vector3 y_d(0.0, 0, -2.0);
Vector3 y_d_dot(0, 0, 0);
Vector3 y_d_ddot(0, 0, 0); // desired motion of the tool tip y

//desired force
//ch_index = 5 + 12;
Vector3 f_d, if_d;
Vector3 tau, cmd_M;
//Motion feedback information
//ch_index = 5 + 12 + 2 ; // translation velocity of the tool tip y
Vector3 y, y_dot, x, x_dot;
Vector3 w; // angular velocity of the quadrotor
double Sw_array[9]={0.0, w.z*(-1.0), w.y,
	 				w.z, 0.0, w.x*(-1.0),
	 				w.y*(-1.0), w.x, 0.0};//skew matrix S(omega)
Matrix33 Sw(Sw_array);

Matrix33 R; //rotation matrix of the quadrotor wrt the fixed frame

// acceleration of the tool tip - in case you want to feed forward this in the force control. In this file, just ignore this
//Variables for true model state from gazebo
Vector3 y_t, y_dot_t, x_t, x_dot_t;
Vector3 w_t;
double Sw_t_array[9]={0.0, w.z*(-1.0), w.y,
	 				w.z, 0.0, w.x*(-1.0),
	 				w.y*(-1.0), w.x, 0.0};//skew matrix S(omega)
Matrix33 Sw_t(Sw_t_array);
Matrix33 R_t;

//--------------------------------------------------
// system parameters
double m = 1.5; // masss
double stp; //timestep
double g = 9.81; //gravity 
double scale_M = 0.26;
double scale_thr = 1.0;
Vector3 e3(0, 0, 1);
double J_array[9]={0.03, 0.0, 0.0,
	 				0.0, 0.05, 0.0,
					0.0, 0.0, 0.1};

Matrix33 J(J_array); //inertia
Vector3 d;
double gamma_r;
double sign_nu_1;
double rho;
double eta = 1.0;
//--------------------------------------------------
// control gains
double b; // the gains
double k;
double k_t; //Attitude control gain
double k_i;
double k_w_d; // gain for controlling omega to omega_d
double nu_3;
Vector3 e_int;

// the main control code here
//void toc_controller(); //Torque controller
void Callback_pos(const geometry_msgs::PoseStamped msg)
{
	 x.x=msg.pose.position.x; //Update Position of UAV fused by FCU
	 x.y=msg.pose.position.y*(-1.0);
	 x.z=msg.pose.position.z*(-1.0);

	 y=x+R*d;
}

void Callback_vel(const geometry_msgs::TwistStamped msg)
{
	 x_dot.x=msg.twist.linear.x; //Update linear velocity of UAV fused by FCU
	 x_dot.y=msg.twist.linear.y*(-1.0);
	 x_dot.z=msg.twist.linear.z*(-1.0);

	 y_dot=x_dot+R*Sw*d;
}

void Callback_gazebo_states(const gazebo_msgs::ModelStates msg)
{
	 x_t.x=msg.pose[1].position.x;
	 x_t.y=msg.pose[1].position.y;
	 x_t.z=msg.pose[1].position.z;
	 
	 double q1 = msg.pose[1].orientation.x;
	 double q2 = msg.pose[1].orientation.y*(-1.0);
	 double q3 = msg.pose[1].orientation.z*(-1.0);
	 double q4 = msg.pose[1].orientation.w;
	 
	 R_t[0] = 1-2*q2*q2-2*q3*q3;
	 R_t[1] = 2*(q1*q2-q3*q4);
	 R_t[2] = 2*(q1*q3+q2*q4);
	 R_t[3] = 2*(q1*q2+q3*q4);
	 R_t[4] = 1-2*q1*q1-2*q3*q3;
	 R_t[5] = 2*(q2*q3-q1*q4);
	 R_t[6] = 2*(q1*q3-q2*q4);
	 R_t[7] = 2*(q2*q3+q1*q4);
	 R_t[8] = 1-2*q1*q1-2*q2*q2;
	 
	 x_dot_t.x=msg.twist[1].linear.x;
	 x_dot_t.y=msg.twist[1].linear.y;
	 x_dot_t.z=msg.twist[1].linear.z;

	 w_t.x=msg.twist[1].angular.x;
	 w_t.y=msg.twist[1].angular.y;
	 w_t.z=msg.twist[1].angular.z;
	 
	 Sw_t[0] = 0;
	 Sw_t[1] = w_t.z*(-1.0);
	 Sw_t[2] = w_t.y;
	 Sw_t[3] = w.z;
	 Sw_t[4] = 0;
	 Sw_t[5] = w_t.x*(-1.0);
	 Sw_t[6] = w_t.y*(-1.0);
	 Sw_t[7] = w_t.x;
	 Sw_t[8] = 0.0;

	 //Calculate tool states using true position/velocity data
	 y_t = x_t + R*d;
	 y_dot_t = x_dot_t + R_t*Sw_t*d;

}

void Callback_imu(const sensor_msgs::Imu msg)
{
	 w.x=msg.angular_velocity.x; //Update state of UAV fused by FCU
	 w.y=msg.angular_velocity.y*(-1.0);
	 w.z=msg.angular_velocity.z*(-1.0);
	 
	 Sw[0] = 0;
	 Sw[1] = w.z*(-1.0);
	 Sw[2] = w.y;
	 Sw[3] = w.z;
	 Sw[4] = 0;
	 Sw[5] = w.x*(-1.0);
	 Sw[6] = w.y*(-1.0);
	 Sw[7] = w.x;
	 Sw[8] = 0.0;
	 
	 double q1 = msg.orientation.x;
	 double q2 = msg.orientation.y*(-1.0);
	 double q3 = msg.orientation.z*(-1.0);
	 double q4 = msg.orientation.w;


	 R[0] = 1-2*q2*q2-2*q3*q3;
	 R[1] = 2*(q1*q2-q3*q4);
	 R[2] = 2*(q1*q3+q2*q4);
	 R[3] = 2*(q1*q2+q3*q4);
	 R[4] = 1-2*q1*q1-2*q3*q3;
	 R[5] = 2*(q2*q3-q1*q4);
	 R[6] = 2*(q1*q3-q2*q4);
	 R[7] = 2*(q2*q3+q1*q4);
	 R[8] = 1-2*q1*q1-2*q2*q2;

}

void Callback_state(const mavros_msgs::State::ConstPtr& msg){
	 current_state = *msg;
}

int main(int argc, char **argv)
{
	 ros::init(argc, argv, "fixedtool");

	 ros::NodeHandle n;
	 //Receive q, w from pixhawk
	 ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, Callback_state);
	 ros::Subscriber sub_imu = n.subscribe("/mavros/imu/data",250, Callback_imu);
	 //Receive position from fcu
	 ros::Subscriber sub_pos = n.subscribe("/mavros/local_position/pose",30,Callback_pos);
	
	 //Receive velocity from fcu
	 ros::Subscriber sub_vel = n.subscribe("/mavros/local_position/velocity",30,Callback_vel);
	 //Command angular velocity to pixhawk
	 ros::Publisher pub_act = n.advertise<mavros_msgs::ActuatorControl>("/fixedtool/actuator_control", 100);
	 ros::Publisher pub_tool = n.advertise<geometry_msgs::PoseStamped>("/fixedtool/tool", loopRate);
	 ros::Publisher pub_tool_v = n.advertise<geometry_msgs::TwistStamped>("/fixedtool/tool_v", loopRate);
	 
	 ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
     ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	 ros::Subscriber sub_gazebo_pos = n.subscribe("/gazebo/model_states", 1000, Callback_gazebo_states);
	 ros::Publisher pub_gazebo_tool = n.advertise<geometry_msgs::PoseStamped>("/fixedtool/gazebo_tool", loopRate);

	 ros::Rate loop_rate(loopRate);
	
	 geometry_msgs::PoseStamped pub_y;
	 geometry_msgs::TwistStamped pub_y_dot;
	 std_msgs::Float64 cmd_thr;

	 //Set Ros parameters
	 n.param("/fixedtool/gains/pos_P",k, 0.3);
	 n.param("/fixedtool/gains/pos_D", b, 0.5);
	 n.param("/fixedtool/gains/att_k", k_t, 1.0);
	 n.param("/fixedtool/gains/att_k_i", k_i, 1.0);
	 n.param("/fixedtool/gains/yaw", nu_3, 1.1);
	 n.param("/fixedtool/tool/d1", d.x, 0.0);
	 n.param("/fixedtool/tool/d2", d.y, 0.0);
	 n.param("/fixedtool/tool/d3", d.z, -0.1);
	 n.param("/fixedtool/mass", m, 1.5);
     n.param("/fixedtool/gains/step", stp, 0.04);
     
	 gamma_r =-1* (d.x/d.z);
	 //double k_star =-nu_3*d.x/d.z; // gain for nu_3 action -  gamma in eq. (16)
	 double u_bar = 2; // gain for nu_3 action -  eto in eq. (16)
	 
	 //Set tool parameters
	 double Sd_array[9] = {0.0, d.z*(-1.0), d.y,
		  					d.z, 0.0, d.x*(-1.0), 
		  					d.y*(-1.0), d.x, 0.0};
	 Matrix33 Sd(Sd_array);
	 
	 double alpha_o = sqrt(d.x*d.x+d.z*d.z); // it is \bar{d} in (8)
	 double delta_o_array[9] = {d.z/alpha_o*(-1.0), 0.0, d.x/alpha_o, 
		  						0.0, 1.0, 0.0,
		  						d.x/alpha_o, 0.0, d.z/alpha_o};
	 Matrix33 delta_o(delta_o_array);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }
	//Initialize actuator control topics
	mavros_msgs::ActuatorControl cmd_actuator;
	cmd_actuator.group_mix = 0;
	cmd_actuator.controls[0]=0.0;
	cmd_actuator.controls[1]=0.0;
	cmd_actuator.controls[2]=0.0;
	cmd_actuator.controls[3]=0.0;
	cmd_actuator.controls[4]=0.0;
	cmd_actuator.controls[5]=0.0;
	cmd_actuator.controls[6]=0.0;
	cmd_actuator.controls[7]=0.0;
	
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_act.publish(cmd_actuator);
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
	
	 int count = 0;
	 while (ros::ok()){
	  if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
		}
		

		  //Calculate desired commands using controller
		  // based on H.-N. Nguyen et al. / Automatica 61 (2015) 289-301

		  //--------------------------------------------------
		  // input
		  // int ch_index = 0;

		  //force feedback channel
		  Vector3 f_e(0.0, 0.0, 0.0);
		  Vector3 if_e(0.0, 0.0, 0.0);
		  
		  //--------------------------------------------------
		  //calculation;

		  Vector3 Rfe = R.Trans()*f_e;
		  Vector3 tau_e = Sd*Rfe;//external torque on the quadrotor - \tau_c eq. (2)
		  // motion control  eq. (4) with this desired control u
		  Vector3 e = y + y_d*(-1.0);
		  Vector3 e_dot = y_dot + y_d_dot*(-1.0);
		  Vector3 u_c = y_d_ddot*m + e_dot*b*(-1.0) + e*k*(-1.0);

		  // the gain in lower force control layer
		  Vector3 nu_l = delta_o*w; //angular velocity transformation - eq. (8)
		  Vector3 u_hat = R.Trans()*(u_c/m*(-1.0) + e3*9.81); //RHS of (9)
		  
		  if(nu_l.x>0.0){
			   sign_nu_1 = 1.0;
		  }
	      else if(nu_l.x>0.0){
			   sign_nu_1 = -1.0;
		  }
		  
		  rho = 2*eta-gamma_r*nu_l.y;

		  Vector3 nu_d;
		  Vector3 nu_d_dot; //calculate \dot{nu} from (15) and (16)

		  nu_d_dot.x = u_hat.y/alpha_o*(-1.0) + nu_l.y*nu_l.z*(-1.0) ;
		  nu_d_dot.y = u_hat.x/d.z*(-1.0)+(nu_l.x*nu_l.x+nu_l.y*nu_l.y)*d.x/d.z + nu_l.x*nu_l.z;
		  nu_d_dot.z = 0.0; // eq. - (9)
		  
		  nu_d = nu_l+nu_d_dot*stp; // integrate nu_d_dot to calculate nu_d
		  nu_d.z = gamma_r*nu_l.x*(1+nu_l.y*nu_l.y)+rho*sign_nu_1; //eq. - (16)

		  //transform nu back to omega
		  Vector3 w_d;
		  Vector3 w_d_dot;

		  w_d = delta_o*nu_d; //Desired angular rate
		  //w_d = 0;
		  w_d_dot = delta_o*nu_d_dot; //Desired angular acceleration
		  
		  e_int = e_int + w_d_dot*stp;
		  
		  tau = Sw*J*w + J*w_d_dot + (w+w_d*(-1.0))*(-1.0)*k_t; //Calculation of moment input
		  //tau = Sw*J*w + J*(w_d_dot +(w-e_int)*(-1.0)*k_t);
		  // tau = Sw*J*w + J*w_d_dot + ((w - w_d)*stp + e_int)*k_i;
		  cmd_M = tau*scale_M; //Scale moment inputs to command inputs

		  double lambda = (u_hat.z - d.x*nu_d_dot.y - d.z*(nu_l.x*nu_l.x+nu_l.y*nu_l.y)+d.x*nu_l.x*nu_l.z)*m;
		  double throttle = lambda/g*0.38-0.15;
		  
		  //Set saturation values
		  if(cmd_M.x>1.0) cmd_M.x = 1.0;
		  else if(cmd_M.x<-1.0) cmd_M.x = -1.0;
		  if(cmd_M.y>1.0) cmd_M.y = 1.0;
		  else if(cmd_M.y<-1.0) cmd_M.y = -1.0;
		  if(cmd_M.z>1.0) cmd_M.z = 1.0;
		  else if(cmd_M.z<-1.0) cmd_M.z = -1.0;

		  if(throttle<0.0) throttle=0.0;//using std::max;
		  else if(throttle>1.0) throttle = 1.0;//using std::min;
		  
		  //Encode Messages ----------------------------------------------------
		  
		  cmd_thr.data=throttle; //Encode Normalized Throttle Value
		  
		  //Encode Command messages for actuator control
		  //Actuator Control is in NED Frame
		  cmd_actuator.group_mix = 0;
		  cmd_actuator.controls[0] = cmd_M.x; //roll
		  cmd_actuator.controls[1] = cmd_M.y*(1.0); //Pitch (East cmd_M.y;
		  cmd_actuator.controls[2] = cmd_M.z*(1.0); //Yaw (Down Pointing)
		  cmd_actuator.controls[3] = cmd_thr.data; //Throttle
		  cmd_actuator.controls[4] = 0.0;
		  cmd_actuator.controls[5] = 0.0;
		  cmd_actuator.controls[6] = 0.0;
		  cmd_actuator.controls[7] = 0.0;
		  
		  //MAVROS variables are ENU coordinates
 		  pub_y.header.stamp = ros::Time::now();//Tool Position in ENU coordinates
		  pub_y.header.seq = count;
		  pub_y.header.frame_id = 1;
		  pub_y.pose.position.x = u_c.x;
		  pub_y.pose.position.y = u_c.y;
		  pub_y.pose.position.z = u_c.z;
		  //pub_y.pose.position.x = y.x;
		  //pub_y.pose.position.y = y.y*(-1.0);
		  //pub_y.pose.position.z = y.z*(-1.0);

		  pub_y_dot.header.stamp = ros::Time::now();
		  pub_y_dot.header.seq = count;
		  pub_y_dot.header.frame_id = 1;
		  pub_y_dot.twist.linear.x = w_d.x;//y_dot.x;
		  pub_y_dot.twist.linear.y = w_d.y;//y_dot.y*(-1.0);
		  pub_y_dot.twist.linear.z = w_d.z;//y_dot.z*(-1.0);


		  //pub_w.publish(cmd_w);
		  //pub_thr.publish(cmd_thr);
		  pub_act.publish(cmd_actuator);
		  pub_tool.publish(pub_y);
		  pub_tool_v.publish(pub_y_dot);
		  ros::spinOnce();
		  loop_rate.sleep();
		  ++count;
	 }
	return 0;
}
