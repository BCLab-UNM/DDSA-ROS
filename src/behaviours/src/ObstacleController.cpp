#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID;

  centerLocation.x = 0;
  centerLocation.y = 0;
}

void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
}

Result ObstacleController::DoWork() {

  clearWaypoints = true;
  result.PIDMode = CONST_PID;

  if(centerSeen){

    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    if(countLeft < countRight) {
      result.pd.cmdAngular = -K_angular;
    } else {
      result.pd.cmdAngular = K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;

  }
  else {

    //obstacle on right side
    if (right < 0.8 || center < 0.8 || left < 0.8) {
      result.type = precisionDriving;

      result.pd.cmdAngular = -K_angular;

      result.pd.setPointVel = 0.0;
      result.pd.cmdVel = 0.0;
      result.pd.setPointYaw = 0;
    }
  }

  return result;
}


void ObstacleController::SetSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;

  ProcessData();
}

void ObstacleController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData() {

  //timeout timer for no tag messages
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if (Td >= 0.5) {
    centerSeen = false;
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

  if (left < triggerDistance || right < triggerDistance || center < triggerDistance || centerSeen) {
    obstacleDetected = true;
    obstacleAvoided = false;
  } else {
    obstacleAvoided = true;
  }
}

void ObstacleController::SetTagData(vector<TagPoint> tags){
  float cameraOffsetCorrection = 0.020; //meters;
  centerSeen = false;
  // this loop is to get the number of center tags
  if (!targetHeld) {
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

  if(obstacleDetected && !obstacleInterrupt) {
    obstacleInterrupt = true;
    return true;
  } else {
    if(obstacleAvoided && obstacleDetected) {
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {
  return !obstacleAvoided;
}

void ObstacleController::SetIgnoreCenter(){
  ignoreCenter = true; //ignore center ultrasound
}

void ObstacleController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::SetTargetHeld() {
  targetHeld = true;
  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}

int ObstacleController::CheckWaypoint(Point InQuestionLocation){
  //cout << "tag: ObstacleController -> step 4:  In CheckingWaypoint"<< endl;
  waypointState = -1;
  this->driveLocation = InQuestionLocation;

  if(IsPointInCircleCenter() < radius*radius){
    cout << " tag:  driveLocation is in the center" << endl;
    waypointState = deleteWaypoints;
  }else if(DoesLineIntersectCircle() < radius){
    cout << "tag:   the path intersects the circle" << endl;
    waypointState = createWaypoints;
  }
  else
  {
    cout << "tag:   Good to Go!" << endl;
    waypointState = continueCurrentWaypoint;
  }

  return waypointState;

}

float ObstacleController::IsPointInCircleCenter(){
  float xVal = (driveLocation.x - centerLocation.x);
  float yVal = (driveLocation.y - centerLocation.y);
  xVal *= xVal;
  yVal *= yVal;
  return xVal + yVal;
}

float ObstacleController::DoesLineIntersectCircle(){
  //First we define the u and v vectors by their components-

    //v is the vector from the current location to the drive point
    float v_x = (driveLocation.x - currentLocation.x);
    float v_y = (driveLocation.y - currentLocation.y);

    //u is the vector from the current location to the center
    float u_x = (centerLocation.x - currentLocation.x);
    float u_y = (centerLocation.y - currentLocation.y);

    //helpful terms- dot product of u and v, and magnitude of v squared
    float u_dot_v = (u_x * v_x) + (u_y * v_y);
    float v_sqrd = (v_x * v_x) + (v_y * v_y);

    //these are the x and y components of u_proj_v
    float u_projected_onto_v_x = v_x * (u_dot_v / v_sqrd);
    float u_projected_onto_v_y = v_y * (u_dot_v / v_sqrd);

    float distance = hypot((centerLocation.x - u_projected_onto_v_x), centerLocation.y - u_projected_onto_v_y);

    return distance;

}

Result ObstacleController::GetAvoidanceWayPoints(Point driveLocation){
  result.type = waypoint;
  result.wpts.waypoints.clear();

/*
  float distance = sqrt(pow((currentLocation.x - driveLocation.x),2) + pow((currentLocation.y - driveLocation.y),2)) / 3;
  Point nextLocation;

  float angle = atan2(driveLocation.y - currentLocation.y, driveLocation.x - currentLocation.x);
  angle += M_PI/2;

  nextLocation.x = currentLocation.x + (distance * cos(angle));
  nextLocation.y = currentLocation.y + (distance * sin(angle));

  cout<< "tag: adding waypoint #1: " << nextLocation.x << ", " << nextLocation.y << endl;
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), nextLocation);

  nextLocation.x = nextLocation.x + (0.75 * cos(angle));
  nextLocation.y = nextLocation.y + (0.75 * sin(angle));

  cout<< "tag: adding waypoint #2: " << nextLocation.x << ", " << nextLocation.y << endl;
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), nextLocation);

  return result;
  */


  //v is the vector from the current location to the drive point
  float v_x = (driveLocation.x - currentLocation.x);
  float v_y = (driveLocation.y - currentLocation.y);

  //u is the vector from the current location to the center
  float u_x = (centerLocation.x - currentLocation.x);
  float u_y = (centerLocation.y - currentLocation.y);

  //helpful terms- dot product of u and v, and magnitude of v squared
  float u_dot_v = (u_x * v_x) + (u_y * v_y);
  float v_sqrd = (v_x * v_x) + (v_y * v_y);

  //these are the x and y components of u_proj_v
  float u_projected_onto_v_x = v_x * (u_dot_v / v_sqrd);
  float u_projected_onto_v_y = v_y * (u_dot_v / v_sqrd);

  float perpendicular_x = -u_projected_onto_v_y;
  float perpendicular_y = u_projected_onto_v_x;

  float perpendicular_mag = hypot(perpendicular_x, perpendicular_y);

  float radius_from_center = sqrt(1.0 + 1.0) + .5;

  Point nextLocation;

  nextLocation.x = centerLocation.x + (perpendicular_x / perpendicular_mag) * radius_from_center;
  nextLocation.y = centerLocation.y + (perpendicular_y / perpendicular_mag) * radius_from_center;

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), nextLocation);
  cout<< "tag: adding waypoint #1: " << nextLocation.x << ", " << nextLocation.y << endl;

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), driveLocation);

  result.PIDMode = FAST_PID;

  return result;



}

bool ObstacleController::hasDetectedObstacle(){
  return obstacleDetected;
}

void ObstacleController::SetCenterLocation(Point centerLocationOdom){
  this->centerLocation = centerLocationOdom;
}
