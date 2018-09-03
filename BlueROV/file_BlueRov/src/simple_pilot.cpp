/*
 * File: bluerov/src/simple_pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends actuator commands to a mavlink controller.
 */

#include <stdio.h>
#include <vector>
#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov/simple_pilotConfig.h>
#include <sensor_msgs/Joy.h>
//visp
#include "visp3/core/vpMatrix.h"

#define ML_DEBUG_MODE

using namespace std;

class Pilot {
  public:
    vpMatrix Map_;
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    // ros::Publisher mavlink_pub;
    ros::ServiceClient command_client;
    ros::Publisher rc_override; // To publish to mavros/rc/override topic
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber hazard_enable_sub;
    ros::Subscriber joy_sub; //to get joystick buttons and axes

    dynamic_reconfigure::Server<bluerov::simple_pilotConfig> server;
    bluerov::simple_pilotConfig config;
    int lights_pins[3]; //pins associated with lights outputs and camera: in this case, pins 9, 10 and 11 (Aux pins)
    float pins_to_set[3]; 
    bool hazards_enabled;
    bool arming; //check if the motors are armed or not 
    float initial_right_light_intensity; //controlling right spot intensity
    float initial_left_light_intensity; // controlling left spot intensity
    //float simultanous_lights_intensity;//controlling simulatenous spots lights intensity
    float initial_camera_position;//camera position starts at 1500
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);//keep track of the joystick controls
    void configCallback(bluerov::simple_pilotConfig &update, uint32_t level);
    void setServo(int index, float pulse_width);
    void setOverrideRCIN(float channel_pitch, float channel_roll, float channel_throttle, float channel_yaw, float channel_forward, float channel_lateral); //New way to control motors rather than set servo
    void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    void hazardCallback(const std_msgs::Bool::ConstPtr& msg);
    void armDisarm(bool armed);//method to hold arm disarm actions
    int  mapValues(float value); //maps [-1,1] to [1100, 1900]
    vpColVector mapValuesCmd(vpColVector vcmd);
    //float camera_pin; //handles camera pin 8
    float boundingLights(float light);//in order to bound light values between -1 and 1
};

Pilot::Pilot() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<bluerov::simple_pilotConfig>::CallbackType f;
  f = boost::bind(&Pilot::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10); //publish on mavros/rc/override topic
  command_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Pilot::velCallback, this);
  hazard_enable_sub = nh.subscribe<std_msgs::Bool>("hazard_enable", 1, &Pilot::hazardCallback, this);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Pilot::joyCallback, this);//subscribe to joy topic to get joystick controls 
  // set initial values
  hazards_enabled = false;
  arming=false; //start with arming false
  lights_pins[0]=8; //the pins associated with light spots on BR2. explained before
  lights_pins[1]=9;
  lights_pins[2]=10; 

  pins_to_set[0] = -1.0; 
  pins_to_set[1] = -1.0;   
  pins_to_set[2] = 0.0;   


  initial_right_light_intensity=-1.0; //start with 0 intensity in all spots
  initial_left_light_intensity=-1.0;
  //simultanous_lights_intensity=-1; 
  initial_camera_position = 0.0;


}

void Pilot::armDisarm(bool armed)
{ //this functions sends a long command service with 400 code to arm or disarm motors
  mavros_msgs::CommandLong srv;
  srv.request.command = 400; //mavros_msgs::CommandLongRequest::CMD_COMPONENENT_ARM_DISARM;
  if(armed)
  {
    srv.request.param1 = 1; // arm motors
  }else{
    srv.request.param1 = 0; // disarm motors
  }
  bool result = command_client.call(srv);
 //ROS_INFO_STREAM("Result" << result);
}

void Pilot::spin() {
  // enforce a max spin rate so we don't kill the CPU
  ros::Rate loop(1000); // Hz

  while(ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();
    loop.sleep();
  }
}


void Pilot::setOverrideRCIN(float channel_pitch, float channel_roll, float channel_throttle, float channel_yaw, float channel_forward, float channel_lateral) {
 //this function replaces setservo for motor commands. It overrides Rc channels inputs and simulates motor controls. In this case, each channel manages a group of motors not individually as servo set.  

  /*

    channel_pitch    = RC_Channels::rc_channel(0);
    channel_roll     = RC_Channels::rc_channel(1);
    channel_throttle = RC_Channels::rc_channel(2);
    channel_yaw      = RC_Channels::rc_channel(3);
    channel_forward  = RC_Channels::rc_channel(4);
    channel_lateral  = RC_Channels::rc_channel(5); 

  */
  vpColVector vcmd(6,0), pulseCmd(6,0);
  vcmd[0] = channel_forward; // vx
  vcmd[1] = channel_lateral; // vy
  vcmd[2] = channel_throttle; // vz
  vcmd[3] = channel_roll; // wx
  vcmd[4] = channel_pitch; // wy
  vcmd[5] = channel_yaw; // wz
  pulseCmd = mapValuesCmd(vcmd);
  
  mavros_msgs::OverrideRCIn msg_override;
  msg_override.channels[0]=pulseCmd[4]; // pitch
  msg_override.channels[1]=pulseCmd[3]; // roll
  msg_override.channels[2]=pulseCmd[2]; // up/down
  msg_override.channels[3]=pulseCmd[5]; // yaw
  msg_override.channels[4]=pulseCmd[0]; // forward
  msg_override.channels[5]=pulseCmd[1]; // lateral
  msg_override.channels[6]=1500;
  msg_override.channels[7]=1500;  
  /*
  msg_override.channels[0]=mapValues(channel_pitch); 
  msg_override.channels[1]=mapValues(channel_roll);
  msg_override.channels[2]=mapValues(channel_throttle);
  msg_override.channels[3]=mapValues(channel_yaw);
  msg_override.channels[4]=mapValues(channel_forward);
  msg_override.channels[5]=mapValues(channel_lateral);
  msg_override.channels[6]=1500;
  msg_override.channels[7]=1500;
  */
  
  rc_override.publish(msg_override);

}

float Pilot::boundingLights(float light)
{ //needs to hold lights intensities between -1 and 1 to be mapped later.
 float mapLight;
 if(light>=1.0)
   mapLight = 1.0; 
 else if(light<=-1.0)
   mapLight=-1.0; 
 else mapLight = light;

return mapLight;
}

int Pilot::mapValues(float value)
{
  int pulse_width = (value+1)*400 + 1100; 
  return pulse_width;
} 

vpColVector Pilot::mapValuesCmd(vpColVector vcmd)
{
  /*vpMatrix Map(6,6,0);
  Map[0][0] = 400;
  Map[1][1] = 400;
  Map[2][2] = 400;
  Map[3][3] = 400;
  Map[4][4] = 400;
  Map[5][5] = 400;*/
  vpColVector zeroValue(6,1500); // override=1500 <=> cmd=0  
  vpColVector pulseVector(6,0);
  #ifdef ML_DEBUG_MODE
  std::cout<<"Map_\n"<<Map_<<std::endl;
  #endif  
  pulseVector = Map_*vcmd + zeroValue;
  //int pulse_width = (value+1)*400 + 1100; 
  //int pulse_width = (value)*400 + 1500; 

  return pulseVector;
} 

void Pilot::configCallback(bluerov::simple_pilotConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;

  ROS_INFO("front_strafe_decouple = %f", config.front_forward_decouple);
  ROS_INFO("front_pitch_biase = %f", config.front_pitch_bias);
  ROS_INFO("front_vertical_bias = %f", config.front_vertical_bias);
  ROS_INFO("buoyancy_control = %f", config.buoyancy_control);
}

void Pilot::setServo(int index, float value) {
 // ROS_INFO_STREAM("setServo");
  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  int pulse_width = (value + 1) * 400 + 1100;

  // send mavros_msgs command message
  // http://docs.ros.org/api/mavros/html/srv/CommandLong.html
  // CMD_DO_SET_SERVO (183): https://pixhawk.ethz.ch/mavlink/
  mavros_msgs::CommandLong srv;
  srv.request.command = 183; //mavros_msgs::CommandLongRequest::CMD_DO_SET_SERVO;
  srv.request.param1 = index + 1; // servos are 1-indexed here
  srv.request.param2 = pulse_width;
 // ROS_INFO_STREAM("before sending to mavros");
  bool result = command_client.call(srv);
  //ROS_INFO_STREAM("Pilot::setServo(" << index << ", " << value << ") = " << result << "pulse_width=" << pulse_width);
}

void Pilot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
 float lights_incremental_factor = 0.1; //factor that scales the steps between pwms.
 //if(!hazards_enabled)return;


	//ROS_INFO("Inside joyCallback.");


 int light_right_increment = joy->buttons[3];  //X button
 int light_right_decrement = joy->buttons[2];  //Y button
 int light_left_increment_decrement  = joy->axes[6];  //right/left axes
 //int light_left_decrement  = joy->buttons[2];  //X  button
 //float simultanous_increment_decrement = joy->axes[6]; //right/left axes
 float camera_increment_decrement = joy->axes[7]; //up/down axes
 
 float btn_arm = joy->buttons[7]; //Start button
 float btn_disarm = joy->buttons[6]; //Back button
 
//arming when Start button is pressed
 if(btn_disarm == 1) {
    if(arming) 
     armDisarm(false);
     arming = false;
 } 
 //disarming when Stop button is pressed
  if(btn_arm == 1) {
    if(!arming) 
     armDisarm(true);
     arming = true;
 } 


//calculates different intensities based on button pressed and actual light intensity in each spot
initial_right_light_intensity +=  ((light_right_increment - light_right_decrement) * lights_incremental_factor);
initial_left_light_intensity  +=  (light_left_increment_decrement  * lights_incremental_factor);
//simultanous_lights_intensity += (simultanous_increment_decrement * lights_incremental_factor);
//simultanous_lights_intensity += (simultanous_increment_decrement * lights_incremental_factor);

initial_camera_position += camera_increment_decrement * lights_incremental_factor;
      

//ensures that different light intensities are between -1 and 1
initial_right_light_intensity = boundingLights(initial_right_light_intensity); 
initial_left_light_intensity = boundingLights(initial_left_light_intensity); 
//simultanous_lights_intensity = boundingLights(simultanous_lights_intensity);
initial_camera_position = boundingLights(initial_camera_position);

//assign to each pin its corresponding light intensity 
 pins_to_set[0] = initial_right_light_intensity;
 pins_to_set[1] = initial_left_light_intensity;
 pins_to_set[2] = initial_camera_position;  


//execute set servo function with calculated lights intensities
 for(int i=0; i<3; i++)
 {
  setServo(lights_pins[i], pins_to_set[i]); 
 }
 
 
  
}

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
//  ROS_INFO_STREAM("velCallback");

  // only continue if hazards are enabled
  if(!hazards_enabled)return;

  // extract cmd_vel message
  /*
  float roll_left_right      = cmd_vel->angular.x;
  float yaw_left_right       = cmd_vel->linear.x;
  float ascend_descend       = cmd_vel->angular.y;
  float forward_reverse      = cmd_vel->angular.z;
  float lateral_left_right   = cmd_vel->linear.y;
  float pitch_left_right     = cmd_vel->linear.z;
  */
  float roll_left_right      = cmd_vel->angular.x;
  float yaw_left_right       = cmd_vel->angular.z;
  float ascend_descend       = cmd_vel->linear.z;
  float forward_reverse      = cmd_vel->linear.x;
  float lateral_left_right   = cmd_vel->linear.y;
  float pitch_left_right     = cmd_vel->angular.y;
  
//
/*
  // build thruster commands (expected to be between -1 and 1)
  float thruster[6];
  thruster[0] =
    roll +
    -config.front_forward_decouple * forward +
    -config.front_strafe_decouple * strafe +
    -config.front_pitch_bias * pitch +
    config.front_vertical_bias * vertical +
    config.buoyancy_control; // Vertical Left (VL)
  thruster[1] =
    config.front_forward_decouple * forward +
    pitch +
    vertical +
    config.buoyancy_control; // Vertical Back (VB)
  thruster[2] =
    -roll +
    -config.front_forward_decouple * forward +
    config.front_strafe_decouple * strafe +
    -config.front_pitch_bias * pitch +
    config.front_vertical_bias * vertical +
    config.buoyancy_control;  // Vertical Right (VR)
  thruster[3] = -yaw + forward; // Forward Left (FL)
  thruster[4] = strafe; // LATeral (LAT)
  thruster[5] = yaw + forward; // Forward Right (FR)
*/
  
  setOverrideRCIN(pitch_left_right,roll_left_right,ascend_descend,yaw_left_right,forward_reverse,lateral_left_right);
  


//ROS_INFO_STREAM("thruster" << yaw);

}

void Pilot::hazardCallback(const std_msgs::Bool::ConstPtr& msg) {
 ROS_INFO_STREAM("hazardCallback"); 
 // save message data
  hazards_enabled = msg->data;
  if(hazards_enabled) 
  {
	ROS_INFO("Enabled thrusters.");
        //arm motors
         if(!arming){
   	 armDisarm(true);
         arming = true;
         }
  }
  else ROS_INFO("Disabled thrusters.");
   
  // zero thruster speeds
  if(!hazards_enabled) {
   //disarm motors
   if(arming){
   armDisarm(false);
   arming = false;
   }
   
setOverrideRCIN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //reset all rc overrides
  for(int i = 0; i < 2; i++) {
     setServo(lights_pins[i], -1.0);	//stop spots lightening	
    }
  setServo(lights_pins[2], 0.0);
  initial_right_light_intensity=-1.0; //reset lights intensities
  initial_left_light_intensity=-1.0; 
  //simultanous_lights_intensity=-1;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  printf("\n\n\n\n\n\n...hello...\n\n\n\n\n\n");    
  Pilot ppilot;
  // input params: mapValuesCmd gain
  ppilot.Map_ = vpMatrix(6,6,0);
  if(argc == 7){   
    ppilot.Map_[0][0] = atof(argv[1]);
    ppilot.Map_[1][1] = atof(argv[2]);
    ppilot.Map_[2][2] = atof(argv[3]);
    ppilot.Map_[3][3] = atof(argv[4]);
    ppilot.Map_[4][4] = atof(argv[5]);
    ppilot.Map_[5][5] = atof(argv[6]);
  }
  else{
    std::cout<<"\n[ WARNING ] using dehaut Map_ matrix\n";
    ppilot.Map_[0][0] = 400;
    ppilot.Map_[1][1] = 400;
    ppilot.Map_[2][2] = 400;
    ppilot.Map_[3][3] = 400;
    ppilot.Map_[4][4] = 400;
    ppilot.Map_[5][5] = 400;
  }
  #ifdef ML_DEBUG_MODE
  std::cout<<"Map_\n"<<ppilot.Map_<<std::endl;
  #endif  
  ppilot.spin();
  return 0;
}
