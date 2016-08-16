#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
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
#include <shared_messages/TagsImage.h>
#include "mobility/rover.h" 

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <queue>
#include <stack>
#include <string>
#include "DDSAController.h"

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng;	

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
std_msgs::Int16 targetDetected; //ID of the detected target
bool targetsCollected [256] = {0}; //array of booleans indicating whether each target ID has been found

//DDSA Numeric Variables

//mobility (package) rover (message type) rover (name of object of message that contains two members a bool MoveToNest and string roverName.
mobility::rover rover; //used to published the rovers names and count the rovers.
bool moveToNest = false;
bool setSpiralLocation = false;
bool setObstacleAvoidance = false;
bool doneWithCollision = false;
std::priority_queue<string, std::vector<string>, std::greater<string> > roversQ;
int numberOfRovers = 0; //count the number of rovers running
int robotNumber;

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
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher roverCountPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;
ros::Subscriber roverCountSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;
ros::Timer roverCountTimer;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const shared_messages::TagsImage::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);
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
  //goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading
      
  targetDetected.data = -1; //initialize target detected

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
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
  targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
  roverCountSubscriber = mNH.subscribe("/numberofrovers", 10, roverCountHandler);

  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
  targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
  targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Int16>((publishedName + "/fingerAngle"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Int16>((publishedName + "/wristAngle"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  roverCountPublish = mNH.advertise<mobility::rover>("/numberofrovers",10,true);

  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  //killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    
  rover.moveToNest = false;
  rover.roverName = publishedName;
  roverCountPublish.publish(rover);
  roverCountTimer = mNH.createTimer(ros::Duration(10.0), roverCountTimerEventHandler, true);
  robotNumber = roverNameToIndex(publishedName);

  ros::spin();
      
  return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
  std_msgs::String stateMachineMsg;

  //How accurate the rover turns are.
  float angle_tol = 0.01;
  
  if (currentMode == 2 || currentMode == 3 ) { //Robot is in automode
    // ROS_INFO_STREAM("RUNNING");
    switch(stateMachineState) {
	
      //Select rotation or translation based on required adjustment
      //If no adjustment needed, select new goal
    case STATE_MACHINE_TRANSFORM: {
      stateMachineMsg.data = "TRANSFORMING";
      //If angle between current and goal is significant
      if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > angle_tol) { // Radians
	ROS_INFO_STREAM("LT ROTATE");
	stateMachineState = STATE_MACHINE_ROTATE; //rotate
      }
      // else if (doneWithCollision == true && setObstacleAvoidance == false){
      // 	  ROS_INFO_STREAM("LT INSIDE 220");
      // 	  if (setSpiralLocation == true){
      // 	    goalLocation = spiralPosition;
      // 	    goalLocation.theta = atan2(spiralPosition.y-currentLocation.y, spiralPosition.x-currentLocation.x);
      // 	    std_msgs::String msg;
      // 	    msg.data = "After obstacle avoidence spiralLocation x: " + boost::lexical_cast<std::string>(spiralPosition.x) + ", " 
      // 	      + "y: " + boost::lexical_cast<std::string>(spiralPosition.y) + ", "
      // 	      + "theta: " + boost::lexical_cast<std::string>(spiralPosition.theta);
      // 	    infoLogPublisher.publish(msg);
      // 	  }
  
      // 	  else{
      // 	    goalLocation.theta = atan2(beforeCollisionPosition.y-currentLocation.y, beforeCollisionPosition.x-currentLocation.x);
      // 	    goalLocation = beforeCollisionPosition;
      // 	  }

      // stateMachineState = STATE_MACHINE_TRANSFORM;// move to translate step
      //	stateMachineState = STATE_MACHINE_ROTATE;
  
      //If goal has not yet been reached
      else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
	ROS_INFO_STREAM("LT TRANSLATE");
	stateMachineState = STATE_MACHINE_TRANSLATE; //translate
	
      }
      //If returning with a target
      else if (targetDetected.data != -1) {
	//If goal has not yet been reached
	if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
	  //set angle to center as goal heading
	  goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
					    
	  //set center as goal position
	  goalLocation.x = 0.0;
	  goalLocation.y = 0.0;
	}
	//Otherwise, reset target and select new random uniform heading
	else {
	  targetDetected.data = -1;
	  //goalLocation.theta = rng->uniformReal(0, 2 * M_PI);  // What is this doing ???? <-----
	}
      }
      //Overrides goalLocation to be goalLocation before the collision happened.
      else if (setObstacleAvoidance==true){
	ROS_INFO_STREAM("LT resetAfterCollision");
      	//	goalLocation.theta = atan2(beforeCollisionPosition.y-currentLocation.y, beforeCollisionPosition.x-currentLocation.x);
      	goalLocation = beforeCollisionPosition;
	std_msgs::String msg;
	msg.data = "After Obstacle avoidence";
	infoLogPublisher.publish(msg); 
	msg.data = "Goal x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
	  + "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
	  + "yaw: " + boost::lexical_cast<std::string>(goalLocation.theta);
	infoLogPublisher.publish(msg);

	//if rover is down correcting inside ROTATING, then reset setObstacleAvoidance to false
	if (doneWithCollision == true) {
	  setObstacleAvoidance = false;
	}
      	
      }
      
      //Otherwise, assign a new goal
      else {
 	ROS_INFO_STREAM("LT NEW GOAL");
	string peekQ; 
	//Handles the situation when the priority queue is empty
	if (roversQ.empty()){
	  peekQ = "test"; 
	}
	else{
	  peekQ = roversQ.top();
	}
	
	if (moveToNest == false && hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.15 && peekQ.compare(publishedName)== 0){
	  
	  //Each rover will wait for 5 seconds before going to the center.
	  ros::Duration(5.0).sleep(); 
	  
	  //set center as goal position
	  goalLocation.x = 0.0;
	  goalLocation.y = 0.0;
	  goalLocation.theta = atan2(0.0-currentLocation.y, 0.0-currentLocation.x);

	}
	//Distance to the center is < 0.15
	else if (moveToNest == false && peekQ.compare(publishedName)== 0){
	 
	  // Makes are rover type of mobility class to tell the rover it is at the center
	  mobility::rover roverUpdate; 
	  moveToNest = true;
	  roverUpdate.roverName = publishedName;
	  roverUpdate.moveToNest = moveToNest;
	  
	  //Publish the name and the moveToNest status.
	  roverCountPublish.publish(roverUpdate);
	}
      
	//Will be called once the rovers finish prioritizing movements to the center at the start
	else if (moveToNest == true){

	  // DDSA Controller needs to know the current location in order to calculate the next goal state
	  ddsa_controller.setX(currentLocation.x);
	  ddsa_controller.setY(currentLocation.y);
            
	  //select new goal state that follows the spiral pattern			      
	  float goalDiffTol = 1.5;//The distance tolerance between goal and spiral to check if rover got back to spiral.
	  if(fabs(goalLocation.x - spiralPosition.x) || fabs(goalLocation.y - spiralPosition.y) <= goalDiffTol){
	    setSpiralLocation = false;//rover is done reaching spiral.
	  }
	  
	  //feel like ddsa gets next location all the time even when the rover is moving to back and forth to collect the food. 
	  //Want to only get next position when down with a pile or cluster. 
	  GoalState gs = ddsa_controller.calcNextGoalState();
	  std_msgs::String msg;
	  msg.data = "x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
	    + "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
	    + "theta: " + boost::lexical_cast<std::string>(goalLocation.theta) + ", "
	    + "dir: " + boost::lexical_cast<std::string>(gs.dir);
	  infoLogPublisher.publish(msg);
	  goalLocation.theta = gs.yaw;
	  goalLocation.x = gs.x;
	  goalLocation.y = gs.y;
	  
	}
	  
	// std_msgs::String msg;
	// msg.data = "x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
	//   + "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
	//   + "theta: " + boost::lexical_cast<std::string>(goalLocation.theta);
	// + "dir: " + boost::lexical_cast<std::string>(gs.dir);
	// infoLogPublisher.publish(msg);
	
	else{

	  if( hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) < 1.5) {
	    //The distance which the rovers move away from its starting position at the start of the stimulation.
	    float radialDistance = 2*hypot(0.0-currentLocation.x, 0.0-currentLocation.y);
	    goalLocation.theta = atan2(currentLocation.y, currentLocation.x);
	    goalLocation.x = radialDistance*cos(goalLocation.theta);
	    goalLocation.y = radialDistance*sin(goalLocation.theta);
	  }
	  else{	 
	    //rovers will face the center once they moved radialDistance away.
	    // goalLocation.theta = atan2(0.0-currentLocation.y, 0.0-currentLocation.x);
	    // goalLocation.x = currentLocation.x;
	    // goalLocation.y = currentLocation.y;
	   
	    //Rovers stops when the rover is not in the priority queue.
	    setVelocity(0.0, 0.0);
	  }
	}
	
      }
    }
				
      //Purposefully fall through to next case without breaking
			
      //Calculate angle between currentLocation.theta and goalLocation.theta
      //Rotate left or right depending on sign of angle
      //Stay in this state until angle is minimized
    case STATE_MACHINE_ROTATE: {
      stateMachineMsg.data = "ROTATING";
      float p_k = 0.25;  
      float angle_error = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
      // float unknown_angle_error = fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x)));
      // float diff_x = currentLocation.x - goalLocation.x;
      // float diff_y = currentLocation.y - goalLocation.y;
      // float diff_x2 = diff_x*diff_x;
      // float diff_y2 = diff_y*diff_y;

      float command_vel = p_k*angle_error;
      if (fabs(angle_error) > angle_tol) {
          
	// std_msgs::String msg;
	// msg.data = "Angle Error: " + boost::lexical_cast<std::string>(angle_error)
	// + ", Unknown Angle Error: " + boost::lexical_cast<std::string>(unknown_angle_error) 
	// + ", Dist: " + boost::lexical_cast<std::string>(sqrt(diff_x2+diff_y2));
	// infoLogPublisher.publish(msg);
	// setObstacleAvoidance = false;
	setVelocity(0.0, command_vel); //rotate
      }
		    	
      else {
	doneWithCollision= true;//rover got out of collision
	setVelocity(0.0, 0.0); //stop	
	stateMachineState = STATE_MACHINE_TRANSLATE;
      }   
      
      break;
    }
      //Calculate angle between currentLocation.x/y and goalLocation.x/y
      //Drive forward
      //Stay in this state until angle is at least PI/2
    case STATE_MACHINE_TRANSLATE: {
      stateMachineMsg.data = "TRANSLATING";
      ROS_INFO_STREAM("LT TRANSLATE");
      if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
	setVelocity(0.3, 0.0);
      }
      else {
	setVelocity(0.0, 0.0); //stop
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
    
  velocity.linear.x = linearVel * 1.5;
  velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
  velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const shared_messages::TagsImage::ConstPtr& message) {

  //if this is the goal target
  if (message->tags.data[0] == 256) {
    //if we were returning with a target
    if (targetDetected.data != -1) {
      //publish to scoring code
      targetDropOffPublish.publish(message->image);
      targetDetected.data = -1;
        
      // Reached the center with target - return to the spiral
      std_msgs::String msg;
      msg.data = "Reached center";
      infoLogPublisher.publish(msg); 
      goalLocation = spiralPosition;
      goalLocation.theta = atan2(spiralPosition.y-currentLocation.y, spiralPosition.x-currentLocation.x);
      msg.data = "Goal x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
	+ "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
	+ "yaw: " + boost::lexical_cast<std::string>(goalLocation.theta) 
	+ " Returning to spiral.";
      infoLogPublisher.publish(msg);

      //rover is going back to spiral, when true prevents robot from collecting target on way back to spiral.
      setSpiralLocation = true;
    }
  }

  //if target has not previously been detected && rover is not going back to spiral.
  else if (targetDetected.data == -1 && setSpiralLocation == false) {
          
    //check if target has not yet been collected
    if (!targetsCollected[message->tags.data[0]]) {
      //copy target ID to class variable
      targetDetected.data = message->tags.data[0];
			
      // Remember where we were in the spiral
      spiralPosition = currentLocation;
      std_msgs::String msg;
      msg.data = "Stored Spiral Position x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
	+ "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
	+ "yaw: " + boost::lexical_cast<std::string>(goalLocation.theta)
	+ "Heading to nest";
      infoLogPublisher.publish(msg);
    
      //set angle to center as goal heading
      goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
	
      //set center as goal position
      goalLocation.x = 0.0;
      goalLocation.y = 0.0;
	
      //publish detected target
      targetCollectedPublish.publish(targetDetected);
      
      //publish to scoring code
      targetPickUpPublish.publish(message->image);
      
      //switch to transform state to trigger return to center
      stateMachineState = STATE_MACHINE_TRANSFORM;
    }   
  }  
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
  if (message->data > 0) {
    beforeCollisionPosition = goalLocation;

    std_msgs::String msg;
    msg.data = "Reached Obstacle";
    infoLogPublisher.publish(msg); 
    msg.data = "Goal x: " + boost::lexical_cast<std::string>(goalLocation.x) + ", " 
      + "y: " + boost::lexical_cast<std::string>(goalLocation.y) + ", "
      + "yaw: " + boost::lexical_cast<std::string>(goalLocation.theta);
    infoLogPublisher.publish(msg);
    
    //variable used to reset rover's goalLocation before obstacle avoidance happened.
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
		
    std_msgs::Int16 angle;
    if (message->axes[6] < 0.) {
      angle.data = -1;
      fingerAnglePublish.publish(angle);
    }
    else if (message->axes[6] > 0.) {
      angle.data = -2;
      fingerAnglePublish.publish(angle);
    }
		
    if (message->axes[7] > 0.) {
      angle.data = -1;
      wristAnglePublish.publish(angle);
    }
    else if (message->axes[7] < 0.) {
      angle.data = -2;
      wristAnglePublish.publish(angle);
    }
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

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
  targetsCollected[message->data] = 1;
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

  //Calls the ddsa controller to generate the pattern (circuits, rovers, robot ID).
  ddsa_controller.generatePattern(10, numberOfRovers, robotNumber);
  std_msgs::String msg;

  //Publishes the pattern for each rover.
  msg.data = "Spiral Pattern: " + ddsa_controller.getPath();
  infoLogPublisher.publish(msg);
      
}
// Determines the unique ID based on the rover name. Only for sim rovers.
int roverNameToIndex( string roverName ) {

  if (publishedName.compare("achilles") == 0){
    return 0;
  }
  else if (publishedName.compare("aeneas") == 0){
    return 1;
  }
  else if (publishedName.compare("ajax") == 0){
    return 2;
  }
  else if (publishedName.compare("diomedes") == 0){
    return 3;
  }
  else if (publishedName.compare("hector") == 0){
    return 4;
  }
  else{
    return 5;
  }
}
