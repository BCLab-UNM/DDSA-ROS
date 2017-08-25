#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID;
  cout << "ObstacleController -> 0" << endl;

}

void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  delay = current_time;
   cout << "ObstacleController -> 1" << endl;
}

Result ObstacleController::DoWork() {
cout << "ObstacleController -> 26" << endl;
  clearWaypoints = true;
  set_waypoint = true;
  result.PIDMode = CONST_PID;

  if(centerSeen){
 cout << "ObstacleController -> 2" << endl;

    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    if(countLeft < countRight) {
      result.pd.cmdAngular = K_angular;
    } else {
      result.pd.cmdAngular = -K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;

  }
  else {
 cout << "ObstacleController -> 3" << endl;

    //obstacle on right side
    if (right < side_trigger_distance || center < center_trigger_distance || left < side_trigger_distance ) {
      result.type = precisionDriving;

      result.pd.cmdAngular = -K_angular;

      result.pd.setPointVel = 0.0;
      result.pd.cmdVel = 0.0;
      result.pd.setPointYaw = 0;
    }
  }

  if (can_set_waypoint) {
 cout << "ObstacleController -> 4" << endl;
    can_set_waypoint = false;
    set_waypoint = false;
    clearWaypoints = false;

    result.type = waypoint;
    result.PIDMode = FAST_PID;
    Point forward;
    forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
    forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(forward);
  }

  return result;
}


void ObstacleController::SetSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;
 cout << "ObstacleController -> 5" << endl;
  ProcessData();
}

void ObstacleController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
   cout << "ObstacleController -> 6" << endl;
}

void ObstacleController::ProcessData() {
 cout << "ObstacleController -> 7" << endl;
  //timeout timer for no tag messages
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if (Td >= 0.5) {
    centerSeen = false;
    phys= false;
    if (!obstacleAvoided)
    {
      can_set_waypoint = true;
    }
  }

  //Process sonar info
  if(ignoreCenter){
    if(center > reactivateCenterThreshold){
      ignoreCenter = false;
    }
    else{
      center = 3;
    }
  }
  else {
    if (center < 0.12) {
      result.wristAngle = 0.8;
    }
    else {
      result.wristAngle = -1;
    }
  }

  if (left < side_trigger_distance || right < side_trigger_distance || center < center_trigger_distance)
  {
    phys = true;
    timeSinceTags = current_time;
  }


  if (centerSeen || phys)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    obstacleAvoided = true;
  }
   cout << "ObstacleController -> 18" << endl;
}

void ObstacleController::SetTagData(vector<TagPoint> tags){
   cout << "ObstacleController -> 19" << endl;
  float cameraOffsetCorrection = 0.020; //meters;
  centerSeen = false;
  countLeft = 0;
  countRight = 0;

  // this loop is to get the number of center tags
  if (!targetHeld) {
     cout << "ObstacleController -> 20" << endl;
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].id == 256) {

        // checks if tag is on the right or left side of the image
        if (tags[i].x + cameraOffsetCorrection > 0) {
          countRight++;

        } else {
          countLeft++;
        }
        centerSeen = true;
        timeSinceTags = current_time;
      }
    }
  }

}

bool ObstacleController::ShouldInterrupt() {

 cout << "ObstacleController -> 21" << endl;
  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
    return true;
  }
  else
  {
    if(obstacleAvoided && obstacleDetected)
    {
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {
   cout << "ObstacleController -> 22" << endl;
  if (can_set_waypoint && set_waypoint)
  {
    return true;
  }

  return !obstacleAvoided;
}

void ObstacleController::SetIgnoreCenter(){
   cout << "ObstacleController -> 23" << endl;
  ignoreCenter = true; //ignore center ultrasound
}

void ObstacleController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
   cout << "ObstacleController -> 24" << endl;
}

void ObstacleController::SetTargetHeld() {
  targetHeld = true;

 cout << "ObstacleController -> 25" << endl;
  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}
