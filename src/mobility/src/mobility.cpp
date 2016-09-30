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

// Goal positions
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D spiralWaypoint;
geometry_msgs::Pose2D collisionAvoidanceWaypoint;
geometry_msgs::Pose2D targetCollectionWaypoint;


int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
bool isTargetDetected = false;
bool isTargetCollected = false;

//DDSA Numeric Variables

//mobility (package) rover (message type) rover (name of object of message that contains two members a bool MoveToNest and string roverName.
mobility::rover rover; //used to published the rovers names and count the rovers.
bool moveToNest = true;
bool setSpiralLocation = false;
bool isAvoidingCollision = false;
bool isCollectingTarget = false;
bool doneWithCollision = false;
std::priority_queue<string, std::vector<string>, std::greater<string> > roversQ;
int numberOfRovers = 0; //count the number of rovers running
int ddsaRoverIndex;
bool ddsaPatternGenerated = false; // So the rovers dont move until they know where they are going

bool singleSearchingForCollectionPointMsgFlag = false;

std::vector<string> roverNames;

// PID variables
float prevAngleError = 0.0;;
float prevPositionError = 0.0;;

// Min and max motor values for the physical rovers
// velocity cap
float min_motor_vel = 0.15;
float max_motor_vel = 0.3;


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
float ddsaGap = 1.0;
DDSAController ddsa_controller(ddsaGap);

bool isCollectionPointFound = false;
geometry_msgs::Pose2D nestLocation;

float positionErrorTol = 0.1; //meters // How close to try and get to goal locations
float angleErrorTol = 0.05;  //rad

int main(int argc, char **argv) {

  gethostname(host, sizeof (host));
  string hostname(host);
  
  rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
    
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
  obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 1, obstacleHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  roverCountSubscriber = mNH.subscribe("/numberofrovers", 10, roverCountHandler);
  
  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  roverCountPublish = mNH.advertise<mobility::rover>("/numberofrovers",10,true);

  spiralWaypoint.x = 0;
  spiralWaypoint.y = 0;
  spiralWaypoint.theta = 0;
  
  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  //killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
  targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true, false);
  
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

  
  roverCountTimer = mNH.createTimer(ros::Duration(1.0), roverCountTimerEventHandler, true); 

  // Initialize the state machine state to transform so we can enter the movement loop
  stateMachineState = STATE_MACHINE_TRANSFORM;
  
  ros::spin();
  
  return EXIT_SUCCESS;
}

// The state machine handles movement toward goal positions
void mobilityStateMachine(const ros::TimerEvent&) {

  std_msgs::String stateMachineMsg;

  // sendInfoLogMsg("state machine");
  
  if (currentMode == 2 || currentMode == 3) { //Robot is in automode

    //  sendInfoLogMsg("auto");
    
    // Update the goalLocation target angle. We want the goal location to point towards the goal x, y coords
    
    // Calculate positional and angle error

    // Uses two distinct paradigms for control. (Bad?)
    // A switch statement tells the rover when to rotate and translate in order to read the next goal location
    // Booleans provide levels of bevahiours with different priorities.
    // The least immediate concern and the default behaviour is following the spiral, then comes collecting targets. The most immediate behaviours
    // that get priority over everything else are avoiding collisions followed by taking collected targets to the nest.
    // follow spiral < pick up targets < take collected target to the nest < avoid collisions 
    
    geometry_msgs::Pose2D nextLocation;

    // Turn until the central collection point (nest) is located.
    // isCollectionPointFound is set to true when an april tag is
    // found with ID 256. The nest location is set to the location of
    // that tag.
    if (!isCollectionPointFound) {
      if (!singleSearchingForCollectionPointMsgFlag){
	singleSearchingForCollectionPointMsgFlag = true;
	sendInfoLogMsg("Searching for collection point...");
        }
        setVelocity(0,0.3);
        return;
    }

    // These behaviours should be ordered by immediacy (a sort of poor mans subsumption archetecture)
    if (isAvoidingCollision) {
      //sendInfoLogMsg("Avoiding collision");
      nextLocation = collisionAvoidanceWaypoint;
    } else if (isTargetCollected) {
      //sendInfoLogMsg("Heading to nest");
      nextLocation = nestLocation;
    } else if (isCollectingTarget) {
      nextLocation = targetCollectionWaypoint;
    } else { // Always follow the spiral if no more pressing behaviour needed
      //sendInfoLogMsg("Following spiral");
      nextLocation = spiralWaypoint; 
    }
    
    float angleToNextLocation = atan2(nextLocation.y - currentLocation.y, nextLocation.x - currentLocation.x);

    float xDiff = nextLocation.x-currentLocation.x;
    float yDiff = nextLocation.y-currentLocation.y;
    float x2 = xDiff*xDiff;
    float y2 = yDiff*yDiff;
    float positionError = sqrt(x2+y2);
    
    float angleError = angles::shortest_angular_distance(currentLocation.theta, angleToNextLocation);

    stringstream ss;
    //ss << "Heading to x=" << nextLocation.x << ", y=" << nextLocation.y << ", angle=" << angleToNextLocation << ", Dist: " << positionError << ", Angle diff: " << angleError << "Current Angle: " << currentLocation.theta; 
    // sendInfoLogMsg(ss.str());
    
    switch(stateMachineState) {
      
      case STATE_MACHINE_TRANSFORM: {
        stateMachineMsg.data = "TRANSFORMING";
        
	if (positionError < positionErrorTol)  { // Current goal reached
	
	  if (isAvoidingCollision) { // Temp goal used to avoid collision

            isAvoidingCollision = false;
            //sendInfoLogMsg("Reached Collision Waypoint");

          } else if (isTargetCollected) {
            sendInfoLogMsg("Reached Nest");
            isTargetCollected = false;
	    //if this is the goal target

            //open fingers to drop off target
            ros::Duration(1, 0).sleep();
            std_msgs::Float32 angle;
            angle.data = M_PI_2;
            fingerAnglePublish.publish(angle);
            
          } else if (isCollectingTarget) {
            sendInfoLogMsg("Reached Target Collection Waypoint");
            //isCollectingTarget = false;
            setVelocity(0,0);
            break;
            
            // If not any of the above then return to the spiral
	  } else { // Ask the DDSA for a new location

	    sendInfoLogMsg("Reached Spiral Waypoint");
            
            ddsa_controller.setX(currentLocation.x); // Rename these functions to represent that they take the previous spiral position
            // not necessarily the previous rover position
            ddsa_controller.setY(currentLocation.y);

            GoalState gs = ddsa_controller.calcNextGoalState();
	    
	    string msg = "New spiral position: " + boost::lexical_cast<std::string>(gs.dir);
	    sendInfoLogMsg(msg);
            
	    spiralWaypoint.x = gs.x;
            spiralWaypoint.y = gs.y;
	    spiralWaypoint.theta = atan2(spiralWaypoint.y - currentLocation.y, spiralWaypoint.x - currentLocation.x);
	  }
         // End of case where a waypoint goal has been reached
	//	sendInfoLogMsg("Transforming: " + ss.str());
        //If angle between current and goal is significant
        } else if (fabs(angleError) > angleErrorTol) {
	  stateMachineState = STATE_MACHINE_ROTATE; //rotate
        //If goal has not yet been reached
      } else if (positionError > positionErrorTol) {
	  stateMachineState = STATE_MACHINE_TRANSLATE; //translate
        }
	
    }

      break;
	
        //Calculate angle between currentLocation.theta and goalLocation.theta
        //Rotate left or right depending on sign of angle
        //Stay in this state until angle is minimized
      case STATE_MACHINE_ROTATE: {
        stateMachineMsg.data = "ROTATING";
	//sendInfoLogMsg("Rotating");
	
	float angleError = angles::shortest_angular_distance(currentLocation.theta, angleToNextLocation);
	
	// Use a PID for more accurate rotations
	float Pk = 0.3;
	float Dk = 0.01;
	float deltaAngleError = prevAngleError-angleError;
	prevAngleError = angleError;
	float cmdVel = Pk*angleError+Dk*deltaAngleError;

	if (fabs(cmdVel) < min_motor_vel ) cmdVel = min_motor_vel*boost::math::sign(cmdVel);

        // velocity cap
        if (cmdVel > max_motor_vel) cmdVel = max_motor_vel;
        if (cmdVel < -max_motor_vel) cmdVel = -max_motor_vel;      

        stringstream ss;
        ss << "AC: " << isAvoidingCollision << ", nx:" << nextLocation.x << ", ny:" << nextLocation.y << ", A:" << angleError << ", D: " << positionError << "C: " << 0.0 << ", " << cmdVel;
        
        //sendInfoLogMsg(ss.str());
	setVelocity(0.0, cmdVel);
	  
	stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step      
      }

	break;
	
        //Calculate angle between currentLocation.x/y and goalLocation.x/y
        //Drive forward
        //Stay in this state until angle is at least PI/2
    case STATE_MACHINE_TRANSLATE: {
      stateMachineMsg.data = "TRANSLATING";
      //sendInfoLogMsg("Translating");

      // check if the current goal has been reached
      float xDiff = nextLocation.x-currentLocation.x;
      float yDiff = nextLocation.y-currentLocation.y;
      float x2 = xDiff*xDiff;
      float y2 = yDiff*yDiff;
      float positionError = sqrt(x2+y2);
      
      // Use a PID for more accurate movement
      float Pk = 1.5;
      float Dk = 0.02;
      float deltaPositionError = prevPositionError-positionError;
      prevPositionError = positionError;
      
      float cmdVel = Pk*positionError+Dk*deltaPositionError;
      
      // Minimum value that actuates the wheels
      if (fabs(cmdVel) < min_motor_vel ) cmdVel = min_motor_vel*boost::math::sign(cmdVel);
      
      // velocity cap
      if (cmdVel > max_motor_vel) cmdVel = max_motor_vel;
      if (cmdVel < -max_motor_vel) cmdVel = -max_motor_vel;      

      /*      
      stringstream ss;
      ss << positionError;
      
      sendInfoLogMsg("Translating: " + ss.str());
      */

      stringstream ss;
      ss << "AC: " << isAvoidingCollision << "nx: " << nextLocation.x << ", ny: " << nextLocation.y << ", A: " << angleError << ", D: " << positionError << "C: " << cmdVel << ", " << 0.0;
      //sendInfoLogMsg(ss.str());
      setVelocity(cmdVel, 0.0);
      
      stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
      
      break;
    }
      
    default: {
      sendInfoLogMsg("Error: unknown state in state machine");
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
  
  //if this is the goal target
  if (message->detections.size() > 0){  
    if (message->detections[0].id == 256){
      if (!isCollectionPointFound) {
      geometry_msgs::PoseStamped tagPose = message->detections[0].pose;
            
      tagPose.header.stamp = ros::Time(0);
      geometry_msgs::PoseStamped odomPose;
      
      // Transform from the camera coordinate system to the odometry coordinate system 

      try {
	tfListener->waitForTransform(publishedName + "/odom", publishedName + "/camera_link", ros::Time(0), ros::Duration(1.0));
	tfListener->transformPose(publishedName + "/odom", tagPose, odomPose);
      }
      catch(tf::TransformException& ex) {
	ROS_INFO("Received an exception trying to transform a point from \"odom\" to \"camera_link\": %s", ex.what());
      }

      float collection_point_x  = currentLocation.x + odomPose.pose.position.x;
      float collection_point_y  = currentLocation.y + odomPose.pose.position.y;

      nestLocation.x = collection_point_x;
      nestLocation.y = collection_point_y;
      
      ddsa_controller.setCollectionPoint(collection_point_x, collection_point_y);
      isCollectionPointFound = true;

      float xDiff = collection_point_x-currentLocation.x;
      float yDiff = collection_point_y-currentLocation.y;
      float x2 = xDiff*xDiff;
      float y2 = yDiff*yDiff;
      float dist = sqrt(x2+y2);
      
      stringstream ss;
      ss << "Collection point: x=" << collection_point_x << ", y=" << collection_point_y << "Dist: " << dist;
      sendInfoLogMsg( ss.str() );
      }
    } else if (message->detections[0].id == 0){


      // Assume we are seeing a target and try to pick it up.
      geometry_msgs::PoseStamped tagPose = message->detections[0].pose;

      tagPose.header.stamp = ros::Time(0);
      geometry_msgs::PoseStamped odomPose;
      
      //sendInfoLogMsg("Moving towards target");
      
      try {
        tfListener->waitForTransform(publishedName + "/odom", publishedName + "/camera_link", ros::Time(0), ros::Duration(1.0));
        tfListener->transformPose(publishedName + "/odom", tagPose, odomPose);
      }
      catch(tf::TransformException& ex) {
        ROS_INFO("Received an exception trying to transform a point from \"odom\" to \"camera_link\": %s", ex.what());
      }

      // Ignore targets that are near the collection point
      float xDiff = currentLocation.x-odomPose.pose.position.x;
      float yDiff = currentLocation.y-odomPose.pose.position.y;
      float x2 = xDiff*xDiff;
      float y2 = yDiff*yDiff;
      float target_dist = sqrt(x2+y2);
      
      // Ignore targets that are near the collection point
      xDiff = nestLocation.x-odomPose.pose.position.x;
      yDiff = nestLocation.y-odomPose.pose.position.y;
      x2 = xDiff*xDiff;
      y2 = yDiff*yDiff;
      float nest_dist = sqrt(x2+y2);
      
      if (nest_dist > 1.0) 
      {
        //if (target_dist > 0.1)
        {
          if(!isTargetCollected) {
            isCollectingTarget = true;
            
            //If no target has been collected, set target pose as goal
            //set goal heading
            targetCollectionWaypoint.theta = atan2(odomPose.pose.position.y - currentLocation.y, odomPose.pose.position.x - currentLocation.x);
            
            //set goal position
            float gripperOffset = 0.20; //m distance between the gripper and the position of the rover base link
            targetCollectionWaypoint.x = odomPose.pose.position.x - ((gripperOffset-positionErrorTol) * cos(targetCollectionWaypoint.theta));
            targetCollectionWaypoint.y = odomPose.pose.position.y - ((gripperOffset-positionErrorTol) * sin(targetCollectionWaypoint.theta));

            stringstream ss;
            ss << target_dist; // << ", nx: " << nestLocation.x << ", ny: " << nestLocation.y << ", tx: " << tagPose.pose.position.x << ", ty: " << tagPose.pose.position.y;   
            //sendInfoLogMsg("Target dist " + ss.str());
            
            //set gripper
            std_msgs::Float32 angle;
            //open fingers
            angle.data = M_PI_2;
            fingerAnglePublish.publish(angle);
            //lower wrist
            wristAnglePublish.publish(angle);
            
            //set state and timeout
            isTargetDetected = true;
            targetDetectedTimer.setPeriod(ros::Duration(5.0));
            targetDetectedTimer.start();
          }
        }
      } else {
        stringstream ss;
        ss << nest_dist; // << ", nx: " << nestLocation.x << ", ny: " << nestLocation.y << ", tx: " << tagPose.pose.position.x << ", ty: " << tagPose.pose.position.y;   
        if (!isTargetCollected) sendInfoLogMsg("Saw target at the collection point. ("+ ss.str() +") Ignoring.");
      }
    } else {
      stringstream ss;
      ss << message->detections[0].id;
      sendInfoLogMsg("Saw an unknown april tag!: " + ss.str());
    }
  }
}
void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
 
    	float avoidanceAngle = 0;
	
	//obstacle on right side
	if (message->data == 1) {
	  isAvoidingCollision = true;     
	  avoidanceAngle = 0.2;
	  //sendInfoLogMsg("Avoiding collision on the right");
          collisionAvoidanceWaypoint.x = currentLocation.x + (0.21 * cos(currentLocation.theta+avoidanceAngle));
          collisionAvoidanceWaypoint.y = currentLocation.y + (0.21 * sin(currentLocation.theta+avoidanceAngle));
	}
	else if (message->data == 2 && !isTargetCollected) { // Check that a target is not causing the middle sensor reading
	  //select new heading to the left
	  //	  sendInfoLogMsg("Avoiding collision");
	  isAvoidingCollision = true;     
	  avoidanceAngle = 0.2; 
	  //sendInfoLogMsg("Avoiding collision in front"); 
	  // multiple collision
          collisionAvoidanceWaypoint.x = currentLocation.x + (0.2 * cos(currentLocation.theta+avoidanceAngle));
          collisionAvoidanceWaypoint.y = currentLocation.y + (0.2 * sin(currentLocation.theta+avoidanceAngle));
          }
	//obstacle on left side
	else if (message->data == 3) {
	  //select new heading to the right
	  //sendInfoLogMsg("Avoiding collision");
	  isAvoidingCollision = true;     
	  avoidanceAngle = -0.2;
          collisionAvoidanceWaypoint.x = currentLocation.x + (0.2 * cos(currentLocation.theta+avoidanceAngle));
          collisionAvoidanceWaypoint.y = currentLocation.y + (0.2 * sin(currentLocation.theta+avoidanceAngle));	
	  //sendInfoLogMsg("Avoiding collision on the left"); 
	}
    	else // no collision
	  {
	    //sendInfoLogMsg("No collision"); 
	    //isAvoidingCollision = false;     
	  }
	//select new position 10 cm from current location
	
	//switch to transform state to trigger collision avoidance
	//stateMachineState = STATE_MACHINE_TRANSFORM;

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
  isTargetDetected = false;

  isTargetCollected = true;
  setVelocity(0,0);
  sendInfoLogMsg("Picking up target");
  std_msgs::Float32 angle;
  angle.data = 0;
  fingerAnglePublish.publish(angle); //close fingers
  ros::Duration(1, 0).sleep();
  wristAnglePublish.publish(angle); //raise wrist
  isCollectingTarget = false;
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
  ddsa_controller.generatePattern(10, numberOfRovers, ddsaRoverIndex);
  ss << " Spiral Pattern for " << " index " << ddsaRoverIndex << ": " << ddsa_controller.getPath();
  ddsaPatternGenerated = true;
  roverCountTimer.stop();
  sendInfoLogMsg(ss.str());
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
    
    int index = find(roverNames.begin(), roverNames.end(), publishedName) - roverNames.begin();
    return index;
}

void sendInfoLogMsg(string message) {
  std_msgs::String msg; 
  msg.data = message;
  infoLogPublisher.publish(msg);
}

