#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//Custom messages

#include "mobility/rover.h" 

#include <apriltags_ros/AprilTagDetectionArray.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <vector>
#include <algorithm> // for sort
#include <queue>
#include <stack>
#include <string>
#include "DDSAController.h"

#include <sstream> // for conversion to strings

using namespace std;

// Store the names of all the rovers so we can assign numbers
// to them based on lexical order

//Random number generator
random_numbers::RandomNumberGenerator* rng;	

void sendInfoLogMsg(string message);

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;

//DDSA Numeric Variables

//mobility (package) rover (message type) rover (name of object of message that contains two members a bool MoveToNest and string roverName.
mobility::rover rover; //used to published the rovers names and count the rovers.
bool moveToNest = true;
bool setSpiralLocation = false;
bool setObstacleAvoidance = false;
bool doneWithCollision = false;
std::priority_queue<string, std::vector<string>, std::greater<string> > roversQ;
int numberOfRovers = 0; //count the number of rovers running
int ddsaRoverIndex;
bool ddsaPatternGenerated = false; // So the rovers dont move until they know where they are going

std::vector<string> roverNames;

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher roverCountPublish;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber roverCountSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;
ros::Timer roverCountTimer;
ros::Timer targetDetectedTimer;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);
void roverCountHandler(const mobility::rover& message);
void roverCountTimerEventHandler(const ros::TimerEvent& event);
void roverPatternDelayTimerEventHandler(const ros::TimerEvent& event);

// Utility functions
int roverNameToIndex(string name);

// Create the DDSA Controller
DDSAController ddsa_controller;
geometry_msgs::Pose2D spiralPosition;
geometry_msgs::Pose2D beforeCollisionPosition;

int main(int argc, char **argv) {

  gethostname(host, sizeof (host));
  string hostname(host);
  
  rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
  goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading
  
  //select initial search position 50 cm from center (0,0)
  goalLocation.x = 0.5 * cos(goalLocation.theta);
  goalLocation.y = 0.5 * sin(goalLocation.theta);
  
  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }
  
  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;
  
  signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly
  
  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
  obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  roverCountSubscriber = mNH.subscribe("/numberofrovers", 10, roverCountHandler);
  
  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  roverCountPublish = mNH.advertise<mobility::rover>("/numberofrovers",10,true);

  
  
  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  //killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
  targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);
  
  tfListener = new tf::TransformListener();
  
  rover.moveToNest = false;
  rover.roverName = publishedName;
  roverCountPublish.publish(rover);

  // Wait for all the rovers to announce themselves
  sendInfoLogMsg(publishedName + " is waiting 10 seconds for all the other rovers to announce themselves");

  for (int i = 0; i < 100; i++ )
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep(); 
  }

  
  roverCountTimer = mNH.createTimer(ros::Duration(10.0), roverCountTimerEventHandler, true);

    
  ros::spin();
  
  return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {

  // Don't move until we know where we are going
  //if (!ddsaPatternGenerated) {
  //  sendInfoLogMsg("Waiting for DDSA pattern");
  //  return;
  // }
    
  std_msgs::String stateMachineMsg;

  //stringstream ss;
  //ss << currentMode;
  //  sendInfoLogMsg("Mode: " + ss.str());
  /*
  if (currentMode == 2 || currentMode == 3) { //Robot is in automode

    switch (stateMachineState){
      case STATE_MACHINE_TRANSFORM: sendInfoLogMsg("Transforming");
        break;
      case STATE_MACHINE_TRANSLATE: sendInfoLogMsg("Translating");
        break;
      case STATE_MACHINE_ROTATE: sendInfoLogMsg("Rotating");
    }
  */
  
    switch(stateMachineState) {
      
      //Select rotation or translation based on required adjustment
      //If no adjustment needed, select new goal
      case STATE_MACHINE_TRANSFORM: {
        stateMachineMsg.data = "TRANSFORMING";
        //If angle between current and goal is significant
        if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
          stateMachineState = STATE_MACHINE_ROTATE; //rotate
        }
        //If goal has not yet been reached
        else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
          stateMachineState = STATE_MACHINE_TRANSLATE; //translate
        }
        //If returning with a target
        else if (targetCollected) {
          //If goal has not yet been reached
          if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
            //set angle to center as goal heading
            goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
            
            //set center as goal position
            goalLocation.x = 0.0;
            goalLocation.y = 0.0;
          }
          //Otherwise, drop off target and select new random uniform heading
          else {
            //open fingers
            std_msgs::Float32 angle;
            angle.data = M_PI_2;
            fingerAnglePublish.publish(angle);
            
            //reset flag
            targetCollected = false;
            
            goalLocation.theta = rng->uniformReal(0, 2 * M_PI);
          }
        }
        //If no targets have been detected, assign a new goal
        else if (!targetDetected) {
          
          // check if the current goal has been reached
          float xDiff = goalLocation.x-currentLocation.x;
          float yDiff = goalLocation.y-currentLocation.y;
          float x2 = xDiff*xDiff;
          float y2 = yDiff*yDiff;
          float dist = sqrt(x2+y2);
          
          stringstream ss;
          ss << dist;
          
          sendInfoLogMsg("Dist to goal: " + ss.str());
          if (dist < 0.5) { // if within half a meter set a new goal
            
            // DDSA Controller needs to know the current location in order to calculate the next goal state
            ddsa_controller.setX(currentLocation.x);
            ddsa_controller.setY(currentLocation.y);
            
            GoalState gs = ddsa_controller.calcNextGoalState();
            string msg = "x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
                + "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
                + "theta: " + boost::lexical_cast<std::string>(goalLocation.theta) + ", "
                + "dir: " + boost::lexical_cast<std::string>(gs.dir);
            sendInfoLogMsg(msg);
            
            goalLocation.theta = gs.yaw;
            goalLocation.x = gs.x;
            goalLocation.y = gs.y;
          }
        }
	
        //Purposefully fall through to next case without breaking
      }
        
        //Calculate angle between currentLocation.theta and goalLocation.theta
        //Rotate left or right depending on sign of angle
        //Stay in this state until angle is minimized
      case STATE_MACHINE_ROTATE: {
        stateMachineMsg.data = "ROTATING";
        if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
          setVelocity(0.0, 0.2); //rotate left
        }
        else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
          setVelocity(0.0, -0.2); //rotate right
        }
        else {
          setVelocity(0.0, 0.0); //stop
          stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step
        }
        break;
      }
        
        //Calculate angle between currentLocation.x/y and goalLocation.x/y
        //Drive forward
        //Stay in this state until angle is at least PI/2
      case STATE_MACHINE_TRANSLATE: {
        stateMachineMsg.data = "TRANSLATING";
        if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
          setVelocity(0.3, 0.0);
        }
        else {
          setVelocity(0.0, 0.0); //stop
          
          //close fingers
          std_msgs::Float32 angle;
          angle.data = 0;
          fingerAnglePublish.publish(angle);
          
          stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
        }
        break;
      }
        
      default: {
        break;
      }
    }
  }
  
  else { // mode is NOT auto
    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";
}

// publish state machine string for user, only if it has changed, though
if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {

  //infoLogPublisher.publish(stateMachineMsg);
        
  stateMachinePublish.publish(stateMachineMsg);
  sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
}
}

void setVelocity(double linearVel, double angularVel) 
{
  // Stopping and starting the timer causes it to start counting from 0 again.
  // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
  // the rover's kill switch wont be called.
  killSwitchTimer.stop();
  killSwitchTimer.start();
  
  velocity.linear.x = linearVel, // * 1.5;
      velocity.angular.z = angularVel; // * 8; //scaling factor for sim; removed by aBridge node
  velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
  
  // If in manual mode do not try to automatically pick up the target
  if (currentMode == 1) return;
  
  // If we saw a target and are not returning to the spiral
  if (message->detections.size() > 0 && setSpiralLocation == false) {
    
    geometry_msgs::PoseStamped tagPose = message->detections[0].pose;
    
    //if target is close enough
    if (hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z) < 0.2) {
      //assume target has been picked up by gripper
      targetCollected = true;
      
      //lower wrist to avoid ultrasound sensors
      std_msgs::Float32 angle;
      angle.data = M_PI_2/4;
      wristAnglePublish.publish(angle);
    }
    
    else {
      tagPose.header.stamp = ros::Time(0);
      geometry_msgs::PoseStamped odomPose;
      
      try {
        tfListener->waitForTransform(publishedName + "/odom", publishedName + "/camera_link", ros::Time(0), ros::Duration(1.0));
        tfListener->transformPose(publishedName + "/odom", tagPose, odomPose);
      }
      
      catch(tf::TransformException& ex) {
        ROS_INFO("Received an exception trying to transform a point from \"odom\" to \"camera_link\": %s", ex.what());
      }
      
      //if this is the goal target
      if (message->detections[0].id == 256) {
        //open fingers to drop off target
        std_msgs::Float32 angle;
        angle.data = M_PI_2;
        fingerAnglePublish.publish(angle);
        goalLocation = spiralPosition;
        goalLocation.theta = atan2(spiralPosition.y-currentLocation.y, spiralPosition.x-currentLocation.x);
      }
      
      //Otherwise, if no target has been collected, set target pose as goal
      else if (!targetCollected) {
        //set goal heading
        goalLocation.theta = atan2(odomPose.pose.position.y - currentLocation.y, odomPose.pose.position.x - currentLocation.x);
        
        //set goal position
        goalLocation.x = odomPose.pose.position.x - (0.26 * cos(goalLocation.theta));
        goalLocation.y = odomPose.pose.position.y - (0.26 * sin(goalLocation.theta));
        
        //set gripper
        std_msgs::Float32 angle;
        //open fingers
        angle.data = M_PI_2;
        fingerAnglePublish.publish(angle);
        //lower wrist
        angle.data = 0.8;
        wristAnglePublish.publish(angle);
        
        //set state and timeout
        targetDetected = true;
        targetDetectedTimer.setPeriod(ros::Duration(5.0));
        targetDetectedTimer.start();

        // Assume we picked up the target
        spiralPosition = currentLocation;

        //switch to transform state to trigger return to center
        stateMachineState = STATE_MACHINE_TRANSFORM;
      }
    }
  }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
  
  if (!targetDetected && (message->data > 0)) {
    beforeCollisionPosition = goalLocation;
    setObstacleAvoidance = true; 
    
    //obstacle on right side
    if (message->data == 1) {
      //select new heading 0.2 radians to the left
      goalLocation.theta = currentLocation.theta + 0.2;
    }
    
    //obstacle in front or on left side
    else if (message->data == 2) {
      //select new heading 0.2 radians to the right
      goalLocation.theta = currentLocation.theta - 0.2;
    }
							
    //select new position 50 cm from current location
    goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
		
    //switch to transform state to trigger collision avoidance
    stateMachineState = STATE_MACHINE_TRANSFORM;
  }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  currentLocation.x = message->pose.pose.position.x;
  currentLocation.y = message->pose.pose.position.y;
	
  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocation.theta = yaw;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {

  if (currentMode == 0 || currentMode == 1) {
    setVelocity(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
  } 

}

void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "DDSA";
  status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
  // No movement commands for killSwitchTime seconds so stop the rover 
  setVelocity(0,0);
  double current_time = ros::Time::now().toSec();
  ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetDetectedReset(const ros::TimerEvent& event) {
  targetDetected = false;
	
  std_msgs::Float32 angle;
  angle.data = 0;
  fingerAnglePublish.publish(angle); //close fingers
  wristAnglePublish.publish(angle); //raise wrist
}

void sigintEventHandler(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

//Pop each rover who finshes moving to the nest at the start of the algorihtm
//And counts the number of rovers runnning in stimulation.
void roverCountHandler(const mobility::rover& message)
{
  roverNames.push_back(message.roverName);
  ddsaRoverIndex = roverNameToIndex(publishedName);

  stringstream ss;
  ss << ddsaRoverIndex;
  sendInfoLogMsg("Assigning " + publishedName + " the number " + ss.str());

  if(message.moveToNest == false){
    roversQ.push(message.roverName);
    numberOfRovers++;
  }
  else{
    roversQ.pop();
  }
}

void roverCountTimerEventHandler(const ros::TimerEvent& event)
{
  //Prints out the priority queue in which the rovers are expected to head to the center at the start of algorithm.
  /*
  std::priority_queue<string, std::vector<string>, std::greater<string> > roversQTemp = roversQ;
  string rovers;
 
  rovers += "(";
  ROS_INFO_STREAM("roversQSize: " << roversQTemp.size());
  while(!roversQTemp.empty()){
    rovers += roversQTemp.top() + ", ";
    roversQTemp.pop();
  }   
  rovers += ")";
  ROS_INFO_STREAM("Qorder: " << rovers);

  */
  
  //Calls the ddsa controller to generate the pattern (circuits, rovers, robot ID).
  stringstream ss;
  ss << "N rovers=" <<  numberOfRovers << ", Rover Index = " << ddsaRoverIndex;
  sendInfoLogMsg("Generating DDSA search pattern with " + ss.str());
  ddsa_controller.generatePattern(10, numberOfRovers, ddsaRoverIndex);
  //infoLogPublisher.publish(publishedName+ " Spiral Pattern: " + ddsa_controller.getPath());
  ddsaPatternGenerated = true;
  roverCountTimer.stop();
}

// Assumes the names of all the rovers in the swarn have been added to
// roverNames.
int roverNameToIndex( string roverName ) {
    // Sort the rover names and assign natural numbers to them based on lexigraphical order
    std::sort(roverNames.begin(), roverNames.end());

    // find our position in the sorted list and therefore our index

    //for (int i = 0; i < roverNames.size(); i++) sendInfoLogMsg(publishedName + ":" + roverNames[i]);

    stringstream ss;
    ss << roverNames.size();
    
    sendInfoLogMsg(publishedName + " heard from " + ss.str() + " rovers");
    
    int index = 1 + find(roverNames.begin(), roverNames.end(), publishedName) - roverNames.begin();
    return index;
}

void sendInfoLogMsg(string message) {
  std_msgs::String msg; 
  msg.data = message;
  infoLogPublisher.publish(msg);
}

