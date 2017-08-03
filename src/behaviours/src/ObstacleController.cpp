#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID;
}

void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
}

Result ObstacleController::DoWork() {

  clearWaypoints = true;

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

int ObstacleController::CheckWaypoint(Point InQuestionLocation, Point centerLocation){
  //cout << "tag: ObstacleController -> step 4:  In CheckingWaypoint"<< endl;
  waypointState = -1;
  this->driveLocation = InQuestionLocation;
  this->centerLocation = centerLocation;

  if(IsPointInCircleCenter() > radius*radius && DoesLineIntersectCircle() > radius ){
    cout << "tag:   drivelocation is not in the center && path from currentLocation to drivelocation does not intersect the circle" << endl;
    waypointState = continueCurrentWaypoint;
  }else if(IsPointInCircleCenter() < radius*radius){
    cout << " tag:  driveLocation is in the center" << endl;
    waypointState = deleteWaypoints;
  }else if(DoesLineIntersectCircle() < radius){
    cout << "tag:   the path intersects the circle" << endl;
    waypointState = createWaypoints;
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
  float LAB = sqrt(pow((currentLocation.x - driveLocation.x),2) + pow((currentLocation.y - driveLocation.y),2));
  float Dx = (currentLocation.x - driveLocation.x)/ LAB;
  float Dy = (currentLocation.y - driveLocation.y)/ LAB;
  float t = Dx * (centerLocation.x - driveLocation.x) + Dy*(centerLocation.y - driveLocation.y);
  float Ex = t * Dx + driveLocation.x;
  float Ey = t * Dy + driveLocation.y;
  float LEC = sqrt(pow((Ex - centerLocation.x), 2) + pow((Ey - centerLocation.y), 2));
  return LEC;

}

Result ObstacleController::GetAvoidanceWayPoints(){
  result.type = waypoint;
  result.wpts.waypoints.clear();
  float distance = sqrt(pow((currentLocation.x - driveLocation.x),2) + pow((currentLocation.y - driveLocation.y),2)) / 3;
  Point nextLocation;

  nextLocation.x = currentLocation.x + (distance + radius * cos(M_PI/2));
  nextLocation.y = currentLocation.y + (distance + radius * sin(M_PI/2));

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), nextLocation);

  nextLocation.x = nextLocation.x + (distance + radius * cos(0));
  nextLocation.y = nextLocation.y + (distance + radius * sin(0));

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), nextLocation);

  return result;


}
