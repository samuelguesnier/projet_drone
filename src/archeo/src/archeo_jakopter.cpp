///// http://ardupilot.org/copter/docs/ac2_guidedmode.html NEED TO CHECKED
///// GUIDED MAY NOT BE THE RIGHT MODE ALWAYS

#include <ros/ros.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/StreamRate.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <cmath>

// SOME UTILITY FLAGS

// VERBOSE_LEVEL
#define CURRENT_VERBOSE 0
#define VERYVERBOSE 3
#define VERBOSE 2
#define ALMOSTVERBOSE 1


// TF2 utilies for the Quaternion managment
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

#define RATIO_Z 1
#define RATIO_Y 1
#define RATIO_X 1
#define MAX_SPEED_CMD 3
#define MAX_VEL_TURN_CMD 45
#define MAVROS_WAITTIMEOUT 10000  // In ms
#define ERROR_ENCOUNTERED -1
#define NO_ERROR_ENCOUNTERED 0
#define TAKEOFF_ALTITUDE 5.0
#define ALTITUDE_TOLERANCE 0.1
#define DELAY_BETWEEN_COMMANDS 1.0

// in meters
#define DEFAULT_DESTINATION_REACHED_TOLERANCE 2.0
/* ##### Specific to navdata ros parameters ##### */
// For the stream rate requests
#define SR_REQUEST_ON (uint8_t)1
#define SR_REQUEST_EXTENDED_STATE_RATE (uint16_t) 1
#define SR_REQUEST_POSITION_RATE (uint16_t)200

// Loop rate in hertz
// For ArDrone, in demo mode it's 15Hz and in normal mode it's 200Hz
#define NAVDATA_DEMO_LOOP_RATE 15
#define NAVDATA_LOOP_RATE 200
#define NAVDATA_DISPLAY_RATE 1000

// Subscribers' buffer size
#define SUB_BUF_SIZE_GLOBAL_POS_REL_ALT 10
#define SUB_BUF_SIZE_BATTERY 10
#define SUB_BUF_SIZE_LOCAL_POS_GP_VEL 10
#define SUB_BUF_SIZE_LOCAL_POS_POSE 10
#define SUB_BUF_SIZE_EXTENDED_STATE 10
#define SUB_BUF_SIZE_STATE 10
#define SUB_BUF_SIZE_CMD_RECEIVED 100
#define SUB_BUF_SIZE_GPS_POSITION 10


/* ##### Specific to navdata (new constants) ##### */
// The value of the battery percentage
#define BATTERY_PERCENTAGE 100  // To put into percentage
#define CRITICAL_BATTERY_LIMIT 10  // 10%

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////      SOME HELPERS     ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


//From http://www.geodatasource.com/developers/c
#define pi 3.14159265358979323846

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  Function prototypes                                           :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double);
double rad2deg(double);

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
  case 'M': //is statute miles (default)
    break;
  case 'K': //kilometers
    dist = dist * 1.609344;
    break;
  case 'N': //nautical miles
    dist = dist * 0.8684;
    break;
  }
  return (dist);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) {
  return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) {
  return (rad * 180 / pi);
}


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////  DATA SHARED WITH LUA  ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

//To protect data accesses for navdata
pthread_mutex_t navdata_lock= PTHREAD_MUTEX_INITIALIZER;

class sharedStaticData {
private:
  float altitude;
  int battery_level;
  unsigned int extended_state_vtol;
  unsigned int extended_state_landed;
  float velocity_x;
  float velocity_y;
  float velocity_z;
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float latitude;
  float longitude;
public:
  mavros_msgs::State current_state;

  sharedStaticData(){
    this->altitude = 0;
    this->battery_level =0;
    this->extended_state_vtol = mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED;
    this->extended_state_landed = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
    this->velocity_x = 0;
    this->velocity_y = 0;
    this->velocity_z = 0;
    this->orientation_x=0;
    this->orientation_y=0;
    this->orientation_z=0;
    this->latitude =0;
    this->longitude=0;
  };

  ~sharedStaticData(){};

  int isConnected(){
    int connect = 0;
    pthread_mutex_lock(&navdata_lock);
    if (current_state.connected) connect= 1;
    pthread_mutex_unlock(&navdata_lock);
    return connect;
  }

  int isArmed(){
    int flag = 0;
    pthread_mutex_lock(&navdata_lock);
    if (current_state.armed) flag= 1;
    pthread_mutex_unlock(&navdata_lock);
    return flag;
  }

  std::string getMode(){
    std::string  mode="";
    pthread_mutex_lock(&navdata_lock);
    mode = current_state.mode;
    pthread_mutex_unlock(&navdata_lock);
    return mode;
  }

  void setAltitudeSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->altitude = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getAltitudeSSD(){
      float v;
      pthread_mutex_lock(&navdata_lock);
      v = this->altitude;
      pthread_mutex_unlock(&navdata_lock);
      return v;
  };

  void setBatteryLevelSSD(int v){
    pthread_mutex_lock(&navdata_lock);
    this->battery_level = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  int getBatteryLevelSSD(){
    int v;
    pthread_mutex_lock(&navdata_lock);
    v = this->battery_level;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setExtendedStateVTOLSSD(int v){
    pthread_mutex_lock(&navdata_lock);
    this->extended_state_vtol = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  int getExtendedStateVTOLSSD(){
    int v;
    pthread_mutex_lock(&navdata_lock);
    v = this->extended_state_vtol;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };


  void setExtendedStateLandedSSD(int v){
    pthread_mutex_lock(&navdata_lock);
    this->extended_state_landed = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  int getExtendedStateLandedSSD(){
    int v;
    pthread_mutex_lock(&navdata_lock);
    v = this->extended_state_landed;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setVelocityXSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->velocity_x = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getVelocityXSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->velocity_x;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setVelocityYSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->velocity_y = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getVelocityYSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->velocity_y;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setVelocityZSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->velocity_z = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getVelocityZSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->velocity_z;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setOrientationXSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->orientation_x = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getOrientationXSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->orientation_x;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setOrientationYSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->orientation_y = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getOrientationYSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->orientation_y;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setOrientationZSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->orientation_z = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getOrientationZSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->orientation_z;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

  void setLatitudeSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->latitude = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getLatitudeSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->latitude;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };


  void setLongitudeSSD(float v){
    pthread_mutex_lock(&navdata_lock);
    this->longitude = v;
    pthread_mutex_unlock(&navdata_lock);
  };
  float getLongitudeSSD(){
    float v;
    pthread_mutex_lock(&navdata_lock);
    v = this->longitude;
    pthread_mutex_unlock(&navdata_lock);
    return v;
  };

};

sharedStaticData SSD;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////  FOR THE MAVROS PART  ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

int loopToDoASecond = NAVDATA_LOOP_RATE;
ros::Rate *loop_rate = NULL;

//////////////////////////////// HELPERS PART ///////////////////////////////////

// usually used after an order
void spinAnumberOfTimes(int n){
  if (!loop_rate) return;
  // Here we'll spin and send navdatas periodically
  while(ros::ok()) {
    // Spin once
    ros::spinOnce(); // activate the callbacks (Navdata)
    // Pause in loop with the given value defined in ros::rate
    loop_rate->sleep();
    n--;
    if (n<0) return;
  }
  ROS_ERROR("spinAnumberOfTime exit because ros not ok\n");
}

//////////////////////////////// NAVDATA PART ///////////////////////////////////


/*!
 * \brief Pikopter navdata ros node
 */
class PikopterNavdata {

  // Public part
public:

  // Public functions
  PikopterNavdata();  // Constructor
  ~PikopterNavdata();  // Destructor
  void sendNavdata();  // Send the navdata
  void display();  // Display the current method of the navdata
  void setBitEndOfBootstrap();
  void askMavrosRate();

  // Handlers
  void getAltitude(const std_msgs::Float64::ConstPtr& msg);
  void handleBattery(const mavros_msgs::BatteryStatus::ConstPtr& msg);
  void handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void getExtendedState(const mavros_msgs::ExtendedState::ConstPtr& msg);
  void getState(const mavros_msgs::State::ConstPtr& msg);
  void handleOrientation(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // Accessors
  bool inDemoMode();

  // Private part
private:

  // Private functions
  void initNavdata();
  void incrementSequenceNumber();
};

/*!
 * \brief Init the navdata buffer
 */
void PikopterNavdata::askMavrosRate() {

  // Check that the service does exist
  if (!ros::service::exists("/mavros/set_stream_rate", true)) {  // Second parameter is whether we print the error or not
    ROS_DEBUG("Can't put the stream rate for navdatas because /mavros/set_stream_rate service is unavailable. Maybe mavros isn't launched yet,we'll wait for it.");
  }

  // We'll wait for it then
  bool mavros_available = ros::service::waitForService("/mavros/set_stream_rate", MAVROS_WAITTIMEOUT);
  if (!mavros_available) {
    ROS_FATAL("Mavros not launched, timeout of %dms reached, exiting...", MAVROS_WAITTIMEOUT);
    delete this;
    exit(ERROR_ENCOUNTERED);
  }

  // Create a StreamRate service handler to call the request
  mavros_msgs::StreamRate sr_ext_status;
  mavros_msgs::StreamRate sr_position;

  // Configure the extended status stream rate request
  sr_ext_status.request.stream_id = mavros_msgs::StreamRateRequest::STREAM_EXTENDED_STATUS;
  sr_ext_status.request.message_rate = SR_REQUEST_EXTENDED_STATE_RATE;
  sr_ext_status.request.on_off = SR_REQUEST_ON;

  // Configure the postion stream rate request
  sr_position.request.stream_id = mavros_msgs::StreamRateRequest::STREAM_POSITION;
  sr_position.request.message_rate = SR_REQUEST_POSITION_RATE;
  sr_position.request.on_off = SR_REQUEST_ON;

  // Call the service for put rate to stream ext_status
  if (ros::service::call("/mavros/set_stream_rate", sr_ext_status))
    ROS_INFO("Mavros extended status rate asked") ;
  else
    ROS_ERROR("Call on set_stream_rate service for extended status failed");

  // Call the service for put rate to stream position
  if (ros::service::call("/mavros/set_stream_rate", sr_position))
    ROS_DEBUG("Mavros position rate asked") ;
  else
    ROS_ERROR("Call on set_stream_rate service for position failed");

}


/*!
 * \brief Constructor of PikopterNavdata
 *
 * \param ip_adress The ip adress on which we create the udp socket
 * \param in_demo True if in demo mode, false if not
 */
PikopterNavdata::PikopterNavdata() {
  if (CURRENT_VERBOSE >= VERYVERBOSE)
    ROS_INFO("PikopterNavdata started");
}

/*!
 * \brief Destructor of PikopterNavdata
 */
PikopterNavdata::~PikopterNavdata() {
}

/*!
 * \brief Function called when a message is published on X node
 */
void PikopterNavdata::getAltitude(const std_msgs::Float64::ConstPtr& msg)  {
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Altitude with value=%f", (float)msg->data);
  SSD.setAltitudeSSD((float)msg->data);
}


/*!
 * \brief Put the battery datas into the navdata
 */
void PikopterNavdata::handleBattery(const mavros_msgs::BatteryStatus::ConstPtr& msg) {
  // Get the value of the battery
  int remaining_battery = (int)(msg->remaining * BATTERY_PERCENTAGE);
  //  if (CURRENT_VERBOSE >= VERYVERBOSE)
  ROS_INFO("Battery with value=%d", remaining_battery);
  SSD.setBatteryLevelSSD(remaining_battery);
}

/*!
 * \brief Get the state of the drone
 */
void PikopterNavdata::getExtendedState(const mavros_msgs::ExtendedState::ConstPtr& msg) {
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Entered getExtendedState");

  // Check if we got strange states
  if ((msg->vtol_state > 0) && (msg->landed_state > 0))
    ROS_WARN("Strange state where the drone is considered as flying and landing at the same time. vtol_state = %d and landed_state = %d", msg->vtol_state, msg->landed_state);
  else if ((msg->vtol_state == 0) && (msg->landed_state == 0))
    ROS_WARN("Strange state where the drone is considered as not flying nor landing. vtol_state = %d and landed_state = %d", msg->vtol_state, msg->landed_state);

  SSD.setExtendedStateVTOLSSD(msg->vtol_state);
  SSD.setExtendedStateLandedSSD(msg->landed_state);
  // Here the managment of the flying state
  switch(msg->vtol_state) {

    // Undefined state (this state can be used when the drone isn't flying but landed)
  case mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  VTOL_STATE_UNDEFINED");break;}

    // When the drone is in transition forward
  case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_FW: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  VTOL_STATE_TRANSITION_TO_FW");break;}

    // ???
  case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_MC: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  VTOL_STATE_TRANSITION_TO_MC");break;}

    // ???
  case mavros_msgs::ExtendedState::VTOL_STATE_MC: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  VTOL_STATE_MC");break;}

    // ???
  case mavros_msgs::ExtendedState::VTOL_STATE_FW: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  VTOL_STATE_FW");break;}
  }


  // If landed
  switch (msg->landed_state) {

    // Undefined state (can be used when the drone isn't landed)
  case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  LANDED_STATE_UNDEFINED");break;}

    // Drone is landed on the ground (based on which altitude he took off)
  case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  LANDED_STATE_ON_GROUND");break;}

    // Drone is landed "in the air" (higher than its take off based altitude)
  case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR: {if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("State is  LANDED_STATE_IN_AIR");break;}
  }

}

/*!
 * \brief Put the velocity datas into the navdata
 *
 * \remark The values got from this function are estimated by the drone.
 *         It could be better to get those values from local_position topics
 *         which fetch those datas form GPS position
 */
void PikopterNavdata::handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Velocity with (x = %f, y = %f, z = %f)", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.y);
  SSD.setVelocityXSSD(msg->twist.linear.x);
  SSD.setVelocityYSSD(msg->twist.linear.y);
  SSD.setVelocityZSSD(msg->twist.linear.z);
}

/*!
 * \brief Put the imu position datas into the navdata
 */
void PikopterNavdata::handleOrientation(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Entered orientation with (x = %f, y = %f, z = %f, w = %f)", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.y, msg->pose.orientation.w);

  SSD.setOrientationXSSD(msg->pose.orientation.x);
  SSD.setOrientationYSSD(msg->pose.orientation.y);
  SSD.setOrientationZSSD(msg->pose.orientation.z);
  // Get the quaternion values
  tf2::Quaternion quaternion (msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  // Cast it as a 3x3 matrix
  tf2::Matrix3x3 matrix (quaternion);

  // Then get the euler values converted from this matrix
  double roll, pitch, yaw;
  matrix.getEulerYPR(yaw, pitch, roll);
  //  navdata_mutex.lock();

  // Updatas velocity datas
  //navdata_current.demo.theta = (float32_t)pitch;
  //navdata_current.demo.phi = (float32_t)roll;
  //navdata_current.demo.psi = (float32_t)yaw;

  /* ##### Exit Critical Section ##### */
  //navdata_mutex.unlock();
}

////// NOT IN THE CLASS X

/*!
 * \brief Getting GPS coordinate
 */
void getGPS(const sensor_msgs::NavSatFix::ConstPtr& fix){
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_WARN("Unable to get a fix on the location.");
    return;
  }
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Current Latitude: %f, Current Longitude %f", fix->latitude, fix->longitude );
  SSD.setLongitudeSSD((float) fix->longitude);
  SSD.setLatitudeSSD((float) fix->latitude);
}



//////////////////////////////// CMD PART ///////////////////////////////////
class ExecuteCommand {
public:
  ExecuteCommand();
  bool takeoff();
  bool land();
  bool rtl();
  bool brake();
  bool stay();
  bool setToGuidedMode();
  bool setToStabilizeMode();
  bool setToLoiterMode();
  bool setToAutoMode();
  void forward(float accel);
  void backward(float accel);
  void down(float accel);
  void up(float accel);
  void left(float accel);
  void right(float accel);
  void slide_left(float accel);
  void slide_right(float accel);
  void move(float roll,float pitch,float upspeed,float yaw);
  void cmd_received();
  void initCmd();
  void waitBetweenCmd();
  void setWaypoint(double lg, double lat, double alt, int frame, int cmd);
  void setWaypointViaMavRosCmd(double lg, double lat, double alt, int frame, int cmd);
  void clearWaypoint();
  ros::Time last_execution_request;

private:
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::ServiceClient takeoff_client;
  ros::ServiceClient land_client;
  ros::ServiceClient mission_client_push;
  ros::ServiceClient mission_client_wp;
  ros::ServiceClient mission_wp_clear;
  ros::ServiceClient command_long_client;
  ros::Publisher velocity_pub;
  ros::Publisher setpoint_raw_pub;
  ros::Publisher navdatas;
  geometry_msgs::TwistStamped msgMove;
  mavros_msgs::PositionTarget msgPosRawPub;
};

mavros_msgs::State current_state;

typedef struct command {
  std::string cmd;
  int seq;
  int tcmd;
  int param1;
  int param2;
  int param3;
  int param4;
  int param5;
} Command;


// Check here https://github.com/PX4/Devguide/blob/master/ros-mavros-offboard.md

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  pthread_mutex_lock(&navdata_lock);
  SSD.current_state = *msg;
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Executing state_cb, updating the state (%s)",SSD.getMode().c_str());
  pthread_mutex_unlock(&navdata_lock);
}

// from https://media.readthedocs.org/pdf/auros/latest/auros.pdf
void wp_cb(const mavros_msgs::WaypointList& wp_list) {
  std::vector<mavros_msgs::Waypoint> waypoints = wp_list.waypoints;
  for(int i = 0; i < waypoints.size(); i++) {
    ROS_INFO("Waypoint loaded @ %f, %f", waypoints[i].x_lat, waypoints[i].y_long);
  }
}


void waitForService(const std::string service) {
  bool mavros_available = ros::service::waitForService(service, MAVROS_WAITTIMEOUT);
  if (!mavros_available) {
    ROS_FATAL("Mavros not launched, timeout of %dms reached, exiting...", MAVROS_WAITTIMEOUT);
    ROS_INFO("Maybe the service you asked does not exist");
    exit(ERROR_ENCOUNTERED);
  }
}

/**
 * Constructor
 * Initialize all mavros services used.
 * Wait for all service to be ready.
 * NodeHandle advertising for all topics used.
 */
ExecuteCommand::ExecuteCommand() {
  last_execution_request = ros::Time::now();
}

void ExecuteCommand::waitBetweenCmd(){
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Starting waitBetweenCmd");

  if (!loop_rate || ! ros::ok()){
    ROS_ERROR("waitBetweenCmd Failed");
    return;
  }
  ros::spinOnce();
  while (ros::ok() && (ros::Time::now() - last_execution_request < ros::Duration(DELAY_BETWEEN_COMMANDS))){
    ros::spinOnce();
    loop_rate->sleep();
  }
}

void ExecuteCommand::clearWaypoint(){
  mavros_msgs::WaypointClear cwp;
  setToGuidedMode();

  mission_wp_clear.call(cwp);
  if (cwp.response.success) {
    ROS_INFO("Waypoint clear ok");
  } else {
    ROS_ERROR("Failed clearing waypoints");
    return;
  }
}

void ExecuteCommand::setWaypointViaMavRosCmd(double lat, double lg, double alt, int frame, int cmd){
  FILE *fp;
  fp=fopen("wayTmp.txt", "w");
  if (!fp) {
    ROS_ERROR("1-Failed loading waypoints from file");
    return;
  }
  fprintf(fp,"QGC WPL 110\n");
  fprintf(fp,"0 \t%d \t%d \t%d \t%f \t%f \t%f \t%f \t%f \t%f \t%f \t%d\n",0,frame,cmd,0.0,0.0,0.0,0.0,lat,lg,alt,1) ;
  fprintf(fp,"1 \t%d \t%d \t%d \t%f \t%f \t%f \t%f \t%f \t%f \t%f \t%d\n",0,frame,cmd,0.0,0.0,0.0,0.0,lat,lg,alt,1) ;
  fclose(fp);
  if (!system("rosrun mavros mavwp load wayTmp.txt\n")){
    ROS_INFO("Waypoint loaded from file");
  }else {
    ROS_ERROR("2-Failed loading waypoints from file");
    return;
  }

}

//see https://github.com/mavlink/mavros/issues/580
void ExecuteCommand::setWaypoint(double lat, double lg, double alt, int frame, int cmd){
  //  std::vector<mavros_msgs::Waypoint> wps;
  mavros_msgs::WaypointPush srv_wpPush;
  mavros_msgs::Waypoint wp;
  mavros_msgs::WaypointSetCurrent wpc;

  setToGuidedMode();

  // http://ardupilot.org/planner/docs/common-mavlink-mission-command-messages-mav_cmd.html
  // Despite the name, this command is really a “NAV_” command rather than a “DO_” command.

  // wp.frame = mavros_msgs::Waypoint::FRAME_LOCAL_NED;//FRAME_GLOBAL; //GLOBAL_FRAME
  // wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT; //NAV_WAYPOINT
  wp.frame = frame;
  wp.command = cmd;
  //wp.is_current = false;
  //wp.autocontinue = false;
  //wp.param1 = 0;
  // http://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html
  //wp.param2 = 2; // Acceptance radius in meters (waypoint is complete when the plane is this close to the waypoint location (Plane only)
  //wp.param3 = 0;
  //wp.param4 = 0;
  wp.x_lat = lat;
  wp.y_long = lg;
  wp.z_alt = alt;
  ROS_INFO("Requesting Waypoint lat=%f, long=%f atl=%f frame=%d cmd=%d",lat,lg,alt,frame,cmd);

  // push it twice, the first waypoint is where the drone is
  srv_wpPush.request.waypoints.push_back(wp);
  mission_client_push.call(srv_wpPush);
  // now this this the real waypoint
  srv_wpPush.request.waypoints.push_back(wp);
  mission_client_push.call(srv_wpPush);

  if (srv_wpPush.response.success) {
    ROS_INFO("Waypoint send ok");
    //now set it as current
    wpc.request.wp_seq = 1;
    mission_client_wp.call(wpc);
    if (wpc.response.success) {
      ROS_INFO("Waypoint set current ok");
      if (system("rosrun mavros mavwp show")){
	ROS_ERROR("Could not print WAYPOINTS list");
      }
      setToAutoMode();

      double ta= alt*ALTITUDE_TOLERANCE;
      if (ta < DEFAULT_DESTINATION_REACHED_TOLERANCE) ta = DEFAULT_DESTINATION_REACHED_TOLERANCE;

      //now lets wait while we are reaching the destination
      while (true){
	double d = 1000.0*distance(SSD.getLatitudeSSD(),SSD.getLongitudeSSD(),lat,lg,'K');
	double a = SSD.getAltitudeSSD() - alt;
	if (a < 0.0) a = -a;

	ROS_INFO("Distance from Waypoint is %f alt diff  is %f",d,a);
	waitBetweenCmd();
	last_execution_request = ros::Time::now();
	if ((d < DEFAULT_DESTINATION_REACHED_TOLERANCE) && (a < ta)) {
	  ROS_INFO("Destination reached");
	  break;
	}
      }
    } else {
      ROS_ERROR("Failed setting current waypoint");
      return;
    }
  }else{
    ROS_ERROR("Failed set waypoint Send");
    return;
  }
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}


void ExecuteCommand::initCmd() {
  ros::NodeHandle nh;

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  command_long_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  mission_client_push = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
  mission_client_wp = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");
  mission_wp_clear = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");


  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for land service");
  waitForService("/mavros/cmd/land");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for takeoff service");
  waitForService("/mavros/cmd/takeoff");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for set_mode service");
  waitForService("/mavros/set_mode");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for set_mode service");
  waitForService("/mavros/cmd/arming");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for commandLong service");
  waitForService("mavros/cmd/command");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for mission  service");
  waitForService("mavros/mission/push");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for mission_wp service");
  waitForService("mavros/mission/set_current");

  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Wait for mission_clear_wp service");
  waitForService("mavros/mission/clear");

  velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);
  navdatas = nh.advertise<std_msgs::Bool>("pikopter_cmd/cmd_received", 100);
  setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

}


/**
 * Takeoff command.
 * First, it will change mode to GUIDED mode.
 * Then it will arm the wehicle.
 * By default the altitude reached after taking off is 10 (meters).
 * If change mode or arming vehicle failed it will return false.
 * Otherwise if the vehicle has taken off it will return true.
 */

bool ExecuteCommand::takeoff() {
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Takeoff asked");
  mavros_msgs::SetMode srvGuided;
  mavros_msgs::CommandTOL srvTakeOffLand;
  mavros_msgs::CommandBool srvArmed;

  srvGuided.request.custom_mode = "GUIDED";
  srvGuided.request.base_mode = 0;
  srvTakeOffLand.request.altitude = TAKEOFF_ALTITUDE;
  srvArmed.request.value = true;

  set_mode_client.call(srvGuided);
  if (srvGuided.response.success) {
    if (CURRENT_VERBOSE >= ALMOSTVERBOSE) ROS_INFO("Guided mode enabled");
  } else {
    ROS_ERROR("Unable to set mode to GUIDED");
    return false;
  }
  //FBO to do that differently
  spinAnumberOfTimes(loopToDoASecond);

  arming_client.call(srvArmed);
  if (srvArmed.response.success) {
    if (CURRENT_VERBOSE >= ALMOSTVERBOSE)
      ROS_INFO("Drone armed");
  } else {
    ROS_ERROR("Unable to arm drone");
    return false;
  }

  //FBO to do that differently
  spinAnumberOfTimes(loopToDoASecond);

  takeoff_client.call(srvTakeOffLand);
  if (srvTakeOffLand.response.success) {
    if (CURRENT_VERBOSE >= ALMOSTVERBOSE) ROS_INFO("Drone flying");
  } else {
    ROS_ERROR("Unable to takeoff");
    return false;
  }

  ROS_INFO("Altitude avant decollage %f\n",SSD.getAltitudeSSD());
  //FBO to do that differently - should respond to an event not to a timer...
  while (SSD.getAltitudeSSD() < TAKEOFF_ALTITUDE - (TAKEOFF_ALTITUDE*ALTITUDE_TOLERANCE)){
    spinAnumberOfTimes(loopToDoASecond/10);
    if (CURRENT_VERBOSE >= ALMOSTVERBOSE) ROS_INFO("Taking OFF altitude is %f\n",SSD.getAltitudeSSD());
    if (!SSD.isArmed()){
      ROS_INFO("PB has disarmed");
      exit(1);
    }
  }
  last_execution_request = ros::Time::now();
  return true;
}

/**
 * Land command.
 * First, it will change mode to GUIDED mode if it's not the case.
 * If change mode it will return false.
 * Otherwise if the vehicle has landed it will return true.

 */
bool ExecuteCommand::land() {
  mavros_msgs::CommandTOL srvTakeOffLand;

  srvTakeOffLand.request.altitude = 0;

  land_client.call(srvTakeOffLand);
  if (srvTakeOffLand.response.success) {
    if (CURRENT_VERBOSE >= ALMOSTVERBOSE)
      ROS_INFO("Drone lands");
  } else {
    ROS_ERROR("Drone cannot land");
    return false;
  }
  while (SSD.getAltitudeSSD() > ALTITUDE_TOLERANCE){
    spinAnumberOfTimes(loopToDoASecond);
    if (CURRENT_VERBOSE >= ALMOSTVERBOSE)
      ROS_INFO("Landing altitude is %f\n",SSD.getAltitudeSSD());
    if (!SSD.isArmed()){
      break;
    }
  }
  last_execution_request = ros::Time::now();
  return true;
}

/**
 * Return-to-Launch command.
 **/
bool ExecuteCommand::rtl() {
  mavros_msgs::SetMode guided_set_mode;
  guided_set_mode.request.custom_mode = "RTL";
  if (SSD.getMode() != "RTL"){
    if( set_mode_client.call(guided_set_mode) &&
	guided_set_mode.response.success){
      ROS_INFO("RTL in ExecuteCommand::RTL() enabled (was %s)",SSD.getMode().c_str());
      waitBetweenCmd();
      last_execution_request = ros::Time::now();
    } else {
      ROS_ERROR("Could not set RTL in ExecuteCommand::RTL()");
    }
  } else {
    ROS_INFO("Already in RTL in ExecuteCommand::RTL() (was %s)",SSD.getMode().c_str());
  }
  //Wait for DISARMED
  while (SSD.isArmed()){
    spinAnumberOfTimes(loopToDoASecond);
  }
  ROS_INFO("DRONE IS DISARMED\n");
  last_execution_request = ros::Time::now();
  return true;
}

/**
 * Return-to-Launch command.
 **/
bool ExecuteCommand::brake() {
  mavros_msgs::SetMode guided_set_mode;
  guided_set_mode.request.custom_mode = "BRAKE";
  if (SSD.getMode() != "BRAKE"){
    if( set_mode_client.call(guided_set_mode) &&
	guided_set_mode.response.success){
      ROS_INFO("BRAKE in ExecuteCommand::BRAKE() enabled (was %s)",SSD.getMode().c_str());
      waitBetweenCmd();
      last_execution_request = ros::Time::now();
    } else {
      ROS_ERROR("Could not set BRAKE in ExecuteCommand::BRAKE()");
    }
  } else {
    if (CURRENT_VERBOSE >= VERYVERBOSE)  ROS_INFO("Already in BRAKE in ExecuteCommand::BRAKE() (was %s)",SSD.getMode().c_str());
  }
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
  return true;
}


bool ExecuteCommand::setToGuidedMode() {
  mavros_msgs::SetMode guided_set_mode;
  guided_set_mode.request.custom_mode = "GUIDED";
  if (SSD.getMode() != "GUIDED"){
    if( set_mode_client.call(guided_set_mode) &&
	guided_set_mode.response.success){
      ROS_INFO("GUIDED in ExecuteCommand::setToGuidedMode() enabled (was %s)",SSD.getMode().c_str());
      waitBetweenCmd();
      last_execution_request = ros::Time::now();
    } else {
      ROS_ERROR("Could not set GUIDED in ExecuteCommand::setToGuidedMode()");
    }
  } else {
    if (CURRENT_VERBOSE >= VERYVERBOSE)  ROS_INFO("Already in GUIDED in ExecuteCommand::setToGuidedMode() (was %s)",SSD.getMode().c_str());
  }
  return true;
}

bool ExecuteCommand::setToAutoMode() {
  mavros_msgs::SetMode auto_set_mode;
  auto_set_mode.request.custom_mode = "AUTO";
  if (SSD.getMode() != "AUTO"){
    if( set_mode_client.call(auto_set_mode) &&
	auto_set_mode.response.success){
      ROS_INFO("AUTO in ExecuteCommand::setToAutoMode() enabled (was %s)",SSD.getMode().c_str());
      waitBetweenCmd();
      last_execution_request = ros::Time::now();
    } else {
      ROS_ERROR("Could not set AUTO in ExecuteCommand::setToAutoMode()");
    }
  } else {
    if (CURRENT_VERBOSE >= VERYVERBOSE)  ROS_INFO("Already in AUTO in ExecuteCommand::setToAutoMode() (was %s)",SSD.getMode().c_str());
  }
  return true;
}


bool ExecuteCommand::setToStabilizeMode() {
  mavros_msgs::SetMode stabilize_set_mode;
  stabilize_set_mode.request.custom_mode = "STABILIZE";
  if (SSD.getMode() != "STABILIZE"){
    if( set_mode_client.call(stabilize_set_mode) &&
	stabilize_set_mode.response.success){
      ROS_INFO("STABILIZE in ExecuteCommand::setToStabilizeMode() enabled (was %s)",SSD.getMode().c_str());
      waitBetweenCmd();
      last_execution_request = ros::Time::now();
    } else {
      ROS_ERROR("Could not set STABILIZE in ExecuteCommand::setToStabilizeMode()");
    }
  } else {
    ROS_INFO("Already in STABILIZE in ExecuteCommand::setToStabilizeMode() (was %s)",SSD.getMode().c_str());
  }
  return true;
}


bool ExecuteCommand::setToLoiterMode() {
  mavros_msgs::SetMode loiter_set_mode;
  loiter_set_mode.request.custom_mode = "LOITER";
  if (SSD.getMode() != "LOITER"){
    if( set_mode_client.call(loiter_set_mode) &&
	loiter_set_mode.response.success){
      ROS_INFO("LOITER in ExecuteCommand::setToLoiterMode() enabled (was %s)",SSD.getMode().c_str());
      waitBetweenCmd();
      last_execution_request = ros::Time::now();
    } else {
      ROS_ERROR("Could not set LOITER in ExecuteCommand::setToLoiterMode()");
    }
  } else {
    ROS_INFO("Already in LOITER in ExecuteCommand::setToLoiterMode() (was %s)",SSD.getMode().c_str());
  }
  return true;
}


/**
 * STAY command.
 * change mode to HOLDPOS.
 */
bool ExecuteCommand::stay() {
  mavros_msgs::SetMode poshold_set_mode;
  poshold_set_mode.request.custom_mode = "POSHOLD";
  if (SSD.getMode() != "POSHOLD"){
    if( set_mode_client.call(poshold_set_mode) &&
	poshold_set_mode.response.success){
      ROS_INFO("POSHOLD in ExecuteCommand::stay() enabled (was %s)",SSD.getMode().c_str());
    } else {
      ROS_ERROR("Could not set POSHOLD in ExecuteCommand::stay()");
    }
  } else {
    ROS_INFO("Already in POSHOLD in ExecuteCommand::stay() (was %s)",SSD.getMode().c_str());
  }
  //
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
  return true;
}

/**
 * Forward command.
 * Given an int corresponding to forward/backaward movement (sent by Jakopter), convert this int to a rate.
 */
void ExecuteCommand::forward(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  //See also http://mavlink.org/messages/common
  //Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
  //NED North East Down // https://en.wikipedia.org/wiki/North_east_down

  msgPosRawPub.coordinate_frame = 8; //FRAME_BODY_NED OK
  msgPosRawPub.type_mask = 0xFC7;

  geometry_msgs::Vector3 vector;

  vector.y = (rate) * ((float) MAX_SPEED_CMD);
  vector.x = 0.0;
  vector.z = 0.0;

  msgPosRawPub.velocity = vector;
  setpoint_raw_pub.publish(msgPosRawPub);

  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}


/**
 * Backward command.
 * Given an int corresponding to forward/backward movement (sent by Jakopter), convert this int to a rate.
 */
void ExecuteCommand::backward(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  msgPosRawPub.coordinate_frame = 8; //FRAME_BODY_NED OK
  msgPosRawPub.type_mask = 0xFC7;

  geometry_msgs::Vector3 vector;

  vector.y = (rate) * ((float) MAX_SPEED_CMD) *(-1.0);
  vector.x = 0.0;
  vector.z = 0.0;

  msgPosRawPub.velocity = vector;
  setpoint_raw_pub.publish(msgPosRawPub);

  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}

/**
 * Allows drone to go down
 */
void ExecuteCommand::down(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  msgMove.twist.linear.z = (rate) * (RATIO_Z);
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
  msgMove.twist.linear.z = 0;
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}

/**
 * Allows drone to go up
 */
void ExecuteCommand::up(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  msgMove.twist.linear.z = (rate) * (RATIO_Z);
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
  msgMove.twist.linear.z = 0;
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();

}

/*
 * Turn to left
 * Maximum : 45 degrees
 */
void ExecuteCommand::left(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  mavros_msgs::CommandLong srvCommand;

  srvCommand.request.command = 115; // MAV_CMD_CONDITION_YAW
  srvCommand.request.confirmation = 0;
  srvCommand.request.param1 = abs((rate) * ((float) MAX_VEL_TURN_CMD));
  srvCommand.request.param3 = -1.0;
  srvCommand.request.param4 = 1.0;

  command_long_client.call(srvCommand);
  if (srvCommand.response.success) {
    if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Turn to left success");
  } else {
    ROS_ERROR("Unable to turn left");
  }
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}


/*
 * Turn to right
 * Maximum : 45 degrees
 */
void ExecuteCommand::right(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  mavros_msgs::CommandLong srvCommand;

  srvCommand.request.command = 115; // MAV_CMD_CONDITION_YAW
  srvCommand.request.confirmation = 0;
  srvCommand.request.param1 = abs((rate) * ((float) MAX_VEL_TURN_CMD));
  srvCommand.request.param3 = 1.0;
  srvCommand.request.param4 = 1.0;

  command_long_client.call(srvCommand);
  if (srvCommand.response.success) {
    if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Turn to right success");
  } else {
    ROS_ERROR("Unable to turn right");
  }
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}


/**
 * Slide to right.
 */
void ExecuteCommand::slide_right(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  msgMove.twist.linear.y = (rate) * (RATIO_Y);
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
  msgMove.twist.linear.y = 0;
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}


/**
 * Slide to left.
 * Minimum : 1 meter
 * Maximum : 10 meters
 */
void ExecuteCommand::slide_left(float accel) {
  float rate = accel;
  if (rate >1.0) rate = 1.0;
  if (rate <0.0) return;
  setToGuidedMode();
  msgMove.twist.linear.y = (rate) * (RATIO_Y) * -1;
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
  msgMove.twist.linear.y = 0;
  velocity_pub.publish(msgMove);
  waitBetweenCmd();
  last_execution_request = ros::Time::now();
}

void ExecuteCommand::move(float roll_i,float pitch_i,float upspeed_i,float yaw_i){
  float roll =roll_i;
  float pitch= pitch_i;
  float upspeed =upspeed_i;
  float  yaw = yaw_i;
  if (roll > 1.0) roll=1.0;
  if (roll < -1.0) roll=-1.0;
  if (pitch > 1.0) pitch=1.0;
  if (pitch < -1.0) pitch=-1.0;
  if (upspeed > 1.0) upspeed=1.0;
  if (upspeed < -1.0) upspeed=-1.0;
  if (yaw > 1.0) yaw=1.0;
  if (yaw < -1.0) yaw=-1.0;

  ROS_INFO("Command Move not yet available");

  //x = cos(yaw)*cos(pitch)
  //y = sin(yaw)*cos(pitch)
  //z = sin(pitch)
}

/*
 * Acknowledgement which allows to send signal to navdatas that a command is sending to the drone
 */
void ExecuteCommand::cmd_received() {
  std_msgs::Bool status;
  status.data = true;
  //navdatas.publish(status);
}


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
///////////////////////////// ROS HANDLE PART //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle *navdata_node_handle = NULL;
ros::NodeHandle *navdata_private_node_handle = NULL; //("~");

ExecuteCommand *executeCommand = NULL;
PikopterNavdata *pn = NULL;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
///////////////////////////// FOR THE LUA PART //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

  // these functions will need to be implemented using MAVROS
  int jakopter_xorientation_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_xorientation_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getOrientationXSSD());
    return 1;
  }

  int jakopter_yorientation_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_yorientation_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getOrientationYSSD());
    return 1;
  }

  int jakopter_zorientation_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_zorientation_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getOrientationZSSD());
    return 1;
  }

  int jakopter_xvelocity_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_xvelocity_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getVelocityXSSD());
    return 1;
  }

    int jakopter_yvelocity_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_yvelocity_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getVelocityYSSD());
    return 1;
  }

  int jakopter_zvelocity_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_zvelocity_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getVelocityZSSD());
    return 1;
  }

  int jakopter_latitude_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_latitude_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getLatitudeSSD());
    return 1;
  }

  int jakopter_longitude_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_longitude_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,SSD.getLongitudeSSD());
    return 1;
  }


  int jakopter_connect_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_connect_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    while (!SSD.isConnected()) {
      ROS_INFO("Waiting for connexion");
      executeCommand->waitBetweenCmd();
    }
    ROS_INFO("Drone is connected");
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_takeoff_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_takeoff_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->takeoff();
    lua_pushinteger(L, 0);
    return 1;
  }

  int jakopter_land_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_land_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_land_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->land();
    lua_pushinteger(L, 0);
    return 1;
  }


  int jakopter_rtl_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_rtl_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_rtl_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->rtl();
    lua_pushinteger(L, 0);
    return 1;
  }

  int jakopter_brake_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_brake_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_brake_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->brake();
    lua_pushinteger(L, 0);
    return 1;
  }

  int jakopter_rotate_left_lua(lua_State* L){
    float angular_speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_rotate_left_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_rotate_left_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->left(angular_speed);
    lua_pushinteger(L, 0);
    return 1;
  }

  int jakopter_rotate_right_lua(lua_State* L){
    float angular_speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_rotate_right_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_rotate_right_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->right(angular_speed);
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_slide_left_lua(lua_State* L){
    float speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_slide_left_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_slide_left_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->slide_left(speed);
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_slide_right_lua(lua_State* L){
    float speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_slide_right_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_slide_right_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->slide_right(speed);
    lua_pushinteger(L,0);
    return 1;
  }


  int jakopter_forward_lua(lua_State* L){
    float speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_forward_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_forward_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->forward(speed);
    lua_pushinteger(L, 0);
    return 1;
  }

  int jakopter_backward_lua(lua_State* L){
    float speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_backward_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_backward_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->backward(speed);
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_up_lua(lua_State* L){
    float speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_up_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_up_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->up(speed);
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_down_lua(lua_State* L){
    float speed = luaL_checknumber(L, 1);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_down_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    if (!SSD.isArmed()) {
      ROS_ERROR("Not ARMED unable to execute jakopter_down_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushinteger(L,0);
    return 1;
  }


  int jakopter_disconnect_lua(lua_State* L){
    if (!ros::ok() || !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_disconnect_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    // Do we have something to do???
    //ros::shutdown();
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_reinit_lua(lua_State* L){
    if (!ros::ok()) {
      ROS_ERROR("Unable to execute jakopter_reinit_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_ftrim_lua(lua_State* L){
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_ftrim_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_calib_lua(lua_State* L){
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_calib_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_move_lua(lua_State* L){
    float roll     = luaL_checknumber(L, 1);
    float forward  = luaL_checknumber(L, 2);
    float vertical = luaL_checknumber(L, 3);
    float yaw  = luaL_checknumber(L, 4);
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_move_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->move(roll,forward,vertical,yaw);
    lua_pushinteger(L, 0);
    return 1;
  }

  int jakopter_stay_lua(lua_State* L){
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_stay_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    executeCommand->stay();
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_emergency_lua(lua_State* L){
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_emergency_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_log_command_lua(lua_State* L){
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_log_command_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushstring(L,0);
    return 1;
  }

  int jakopter_is_flying_lua(lua_State* L){
    if (!ros::ok()) {
      ROS_ERROR("Unable to execute jakopter_is_flying_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushinteger(L,0);
    return 1;
  }

  int jakopter_height_lua(lua_State* L){
    if (!ros::ok()|| !executeCommand) {
      ROS_ERROR("Unable to execute jakopter_height_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushnumber(L,0);
    return 1;
  }

  int jakopter_log_navdata_lua(lua_State* L){
    if (!ros::ok()) {
      ROS_ERROR("Unable to execute jakopter_log_navdata_lua");
      lua_pushinteger(L,1);
      return 1;
    }
    lua_pushstring(L,0);
    return 1;
  }

  int usleep_lua(lua_State* L){
    lua_Integer duration = luaL_checkinteger(L, 1);
    //http://man7.org/linux/man-pages/man3/usleep.3.html
    usleep(duration);
    return 0;
  }

  int yield_lua(lua_State* L){
    //sched_yield();
    return 0;
  }

int jakopter_battery_lua(lua_State* L){
  if (!ros::ok()) {
    ROS_ERROR("Unable to execute jakopter_battery_lua");
    lua_pushinteger(L,1);
    return 1;
  }
  lua_pushinteger(L, SSD.getBatteryLevelSSD());
  return 1;
}

int jakopter_distance_lua(lua_State* L){
    float  lat1    = luaL_checknumber(L, 1);
    float lon1  = luaL_checknumber(L, 2);
    float lat2 = luaL_checknumber(L, 3);
    float lon2 = luaL_checknumber(L, 4);

    printf("lat1 %f, lon1 %f, lat2 %f, lon2 %f\n",lat1,lon1,lat2,lon2);
    lua_pushnumber(L,(float) 1000.0*distance(lat1,lon1,lat2,lon2,'K'));
    return 1;
}

  int jakopter_clearwaypoint_lua(lua_State* L){
    executeCommand->clearWaypoint();
    lua_pushnumber(L,0);
    return 1;
  }


  int jakopter_waypoint_lua(lua_State* L){
    float  lat    = luaL_checknumber(L, 1);
    float lon  = luaL_checknumber(L, 2);
    float alt = luaL_checknumber(L, 3);
    int frame = luaL_checkinteger(L, 4);
    int cmd = luaL_checkinteger(L, 5);

    executeCommand->setWaypoint(lat,lon,alt,frame,cmd);
    //executeCommand->setWaypointViaMavRosCmd(lat,lon,alt,frame,cmd);
    lua_pushnumber(L,0);
    return 1;
  }

static int create_cleanup_udata(lua_State* L){
  /*    //metatable with cleanup method for the lib
    luaL_newmetatable(L, "jakopter.cleanup");
    //set our cleanup method as the __gc callback
    lua_pushstring(L, "__gc");
    lua_pushcfunction(L, jakopter_cleanup_lua);
    lua_settable(L, -3);
    //use a userdata to hold our cleanup function.
    //  We don't store anything in it, so its size is meaningless.
    lua_pushstring(L, "jakopter.cleanup_dummy");
    lua_newuserdata(L, 4);
    luaL_getmetatable(L, "jakopter.cleanup");
    lua_setmetatable(L, -2);
    //store this dummy data in Lua's registry.
    lua_settable(L, LUA_REGISTRYINDEX);
    */
    return 0;
  }


static int import_globals(lua_State* L){
  //    lua_pushinteger(L, CHANNEL_NETWORK_OUTPUT);
  //   lua_setglobal(L, "chan_net_output");
    return 0;
  }


int luaopen_libjakopter(lua_State* L) {
    create_cleanup_udata(L);
    import_globals(L);
    lua_newtable(L);

    lua_pushcfunction(L,jakopter_connect_lua);
    lua_setglobal(L, "connect");
    lua_pushcfunction(L,jakopter_disconnect_lua);
    lua_setglobal(L, "disconnect");
    lua_pushcfunction(L,jakopter_takeoff_lua);
    lua_setglobal(L, "takeoff");
    lua_pushcfunction(L,jakopter_land_lua);
    lua_setglobal(L, "land");
    lua_pushcfunction(L,jakopter_rtl_lua);
    lua_setglobal(L, "rtl");
    lua_pushcfunction(L,jakopter_brake_lua);
    lua_setglobal(L, "brake");
    lua_pushcfunction(L,jakopter_emergency_lua);
    lua_setglobal(L, "emergency");
    lua_pushcfunction(L,jakopter_rotate_left_lua);
    lua_setglobal(L, "left");
    lua_pushcfunction(L,jakopter_rotate_right_lua);
    lua_setglobal(L, "right");
    lua_pushcfunction(L,jakopter_slide_left_lua);
    lua_setglobal(L, "slide_left");
    lua_pushcfunction(L,jakopter_slide_right_lua);
    lua_setglobal(L, "slide_right");
    lua_pushcfunction(L,jakopter_forward_lua);
    lua_setglobal(L, "forward");
    lua_pushcfunction(L,jakopter_backward_lua);
    lua_setglobal(L, "backward");
    lua_pushcfunction(L,jakopter_up_lua);
    lua_setglobal(L, "up");
    lua_pushcfunction(L,jakopter_down_lua);
    lua_setglobal(L, "down");
    lua_pushcfunction(L,jakopter_stay_lua);
    lua_setglobal(L, "stay");
    lua_pushcfunction(L,jakopter_move_lua);
    lua_setglobal(L, "move");
    lua_pushcfunction(L,jakopter_reinit_lua);
    lua_setglobal(L, "reinit");
    lua_pushcfunction(L,jakopter_ftrim_lua);
    lua_setglobal(L, "ftrim");
    lua_pushcfunction(L,jakopter_calib_lua);
    lua_setglobal(L, "calib");
    lua_pushcfunction(L,jakopter_log_command_lua);
    lua_setglobal(L, "log_command");
    lua_pushcfunction(L,jakopter_is_flying_lua);
    lua_setglobal(L, "is_flying");
    lua_pushcfunction(L,jakopter_battery_lua);
    lua_setglobal(L, "battery");
    lua_pushcfunction(L,jakopter_height_lua);
    lua_setglobal(L, "height");
    lua_pushcfunction(L,jakopter_log_navdata_lua);
    lua_setglobal(L, "log_navdata");
    lua_pushcfunction(L,usleep_lua);
    lua_setglobal(L, "usleep");
    lua_pushcfunction(L,yield_lua);
    lua_setglobal(L, "yield");
    lua_pushcfunction(L,jakopter_battery_lua);
    lua_setglobal(L, "battery");
    lua_pushcfunction(L,jakopter_latitude_lua);
    lua_setglobal(L, "latitude");
    lua_pushcfunction(L,jakopter_longitude_lua);
    lua_setglobal(L, "longitude");
    lua_pushcfunction(L,jakopter_zvelocity_lua);
    lua_setglobal(L, "zvelocity");
    lua_pushcfunction(L,jakopter_yvelocity_lua);
    lua_setglobal(L, "yvelocity");
    lua_pushcfunction(L,jakopter_xvelocity_lua);
    lua_setglobal(L, "xvelocity");
    lua_pushcfunction(L,jakopter_zorientation_lua);
    lua_setglobal(L, "zorientation");
    lua_pushcfunction(L,jakopter_yorientation_lua);
    lua_setglobal(L, "yorientation");
    lua_pushcfunction(L,jakopter_xorientation_lua);
    lua_setglobal(L, "xorientation");
    lua_pushcfunction(L,jakopter_distance_lua);
    lua_setglobal(L, "distance");
    lua_pushcfunction(L,jakopter_waypoint_lua);
    lua_setglobal(L, "waypoint");
        lua_pushcfunction(L,jakopter_clearwaypoint_lua);
    lua_setglobal(L, "clearwaypoint");

    return 0;
}

}//end extern

lua_State *L = NULL;

/**
 * Execute the lua script
 */
void executeLuaScript(const char *script){
  lua_State *L = luaL_newstate();
  int retLuaState =  luaopen_libjakopter(L);
  if (retLuaState){
    fprintf(stderr,"1-Lua Error %d\n",retLuaState);
  }
  luaL_openlibs(L);
  if (script == NULL) {
    ROS_INFO("Executing flight /home/guesnier/catkin_ws/flight.lua");
    retLuaState = luaL_dofile(L, "/home/guesnier/catkin_ws/flight.lua");
  } else {
    retLuaState = luaL_dostring(L,script);
  }
  if (retLuaState){
    fprintf(stderr,"2-Lua Error %d\n",retLuaState);
  }
  lua_close(L);
  L = NULL;
}



int main(int argc, char **argv){
  ros::init(argc, argv, "jakopter");
  if (CURRENT_VERBOSE >= VERYVERBOSE) ROS_INFO("Navdata node initialized with a rate of %u", NAVDATA_LOOP_RATE);
  navdata_node_handle = new ros::NodeHandle();
  navdata_private_node_handle = new ros::NodeHandle("~");
  loop_rate = new ros::Rate(NAVDATA_LOOP_RATE);

  pn = new PikopterNavdata();
  executeCommand = new ExecuteCommand();;
  // Ask mavros the rate on which it wants to receive the datas
  if (pn) pn->askMavrosRate();  // Will wait mavros to be launched before continuing the execution
  executeCommand->initCmd();
  // Here we receive the navdatas from pikopter_mavlink
  ros::Subscriber sub_mavros_global_position_rel_alt = navdata_node_handle->subscribe("mavros/global_position/rel_alt",
										     SUB_BUF_SIZE_GLOBAL_POS_REL_ALT, &PikopterNavdata::getAltitude, pn);

  // Here we receive the battery state
  // There is an issue here Client [/jakopter] wants topic /mavros/battery to have
  // datatype/md5sum [mavros_msgs/BatteryStatus/8ba4ae7c602c5ae6a7e8f3ffb652dc5f], but our version has [sensor_msgs/BatteryStatus/476f837fa6771f6e16e3bf4ef96f8770].
  // We keep it but this is not activated
  //ros::Subscriber sub_mavros_battery = navdata_node_handle->subscribe("mavros/battery", SUB_BUF_SIZE_BATTERY, &PikopterNavdata::handleBattery, pn);

  // Here we receive the velocity
  ros::Subscriber sub_mavros_local_position_gp_vel = navdata_node_handle->subscribe("mavros/local_position/velocity",
										   SUB_BUF_SIZE_LOCAL_POS_GP_VEL, &PikopterNavdata::handleVelocity, pn);

  // Here we receive the imu position
  ros::Subscriber sub_mavros_local_position_pose = navdata_node_handle->subscribe("mavros/local_position/pose",
										 SUB_BUF_SIZE_LOCAL_POS_POSE, &PikopterNavdata::handleOrientation, pn);

  // Here we receive the state of the drone -- DOES NOT WORK, STATE TOPIC DOES NOT EMIT ANYTHING
  ros::Subscriber sub_mavros_extended_state = navdata_node_handle->subscribe("mavros/extended_state",
									     SUB_BUF_SIZE_EXTENDED_STATE, &PikopterNavdata::getExtendedState,pn);

  // Here we should have the state
  ros::Subscriber sub_mavros_state = navdata_node_handle->subscribe("mavros/state",SUB_BUF_SIZE_EXTENDED_STATE, state_cb);


  // Here we receive the GPS position
  ros::Subscriber sub_mavros_gps_position = navdata_node_handle->subscribe("mavros/global_position/global",
									   SUB_BUF_SIZE_GPS_POSITION,getGPS);

  //from https://media.readthedocs.org/pdf/auros/latest/auros.pdf 4.1.2
  // Current waypoint table. Updates on changes.
  ros::Subscriber sub_mavros_wplist = navdata_node_handle->subscribe("/fcu/mission/waypoints", 1, wp_cb);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// HERE WE CAN START THE EXECUTION OF THE SCRIPT //////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //executeLuaScript("connect() takeoff() up(1) up(1) stay() stay() up(1) up(1) up(1) up(1) stay() stay() stay() stay() down(1) down(1) down(1) down(1) down(1) land()");
  executeLuaScript(NULL);

  //wait before leaving // DEBUG SETING
  for (int i =0; i< 100; i++){
    executeCommand->waitBetweenCmd();
  }
  return NO_ERROR_ENCOUNTERED;
}
