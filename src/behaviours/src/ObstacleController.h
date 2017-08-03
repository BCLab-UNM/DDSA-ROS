#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif
#include <math.h>
#include "Controller.h"
#include "TagPoint.h"

class ObstacleController : virtual Controller
{
public:
  ObstacleController();

  Result result;

  void Reset() override;
  Result DoWork() override;
  void SetSonarData(float left, float center, float right);
  void SetCurrentLocation(Point currentLocation);
  void SetTagData(vector<TagPoint> tags);
  bool ShouldInterrupt() override;
  bool HasWork() override;
  void SetIgnoreCenter();
  void SetCurrentTimeInMilliSecs( long int time );
  void SetTargetHeld ();
  void SetTargetHeldClear() {targetHeld = false; previousTargetState = false;}
  bool GetShouldClearWaypoints() {bool tmp = clearWaypoints; clearWaypoints = false; return tmp;}
  int CheckWaypoint(Point InQuestionLocation, Point centerLocation);
  float IsPointInCircleCenter();
  float DoesLineIntersectCircle();
  Result GetAvoidanceWayPoints();
protected:

  void ProcessData();

private:

  const float K_angular = 1.0; //radians a second
  const float reactivateCenterThreshold = 0.8;
  const int targetCountPivot = 6;
  const float obstacleDistancePivot = 0.2526;
  const float triggerDistance = 0.8;
  const int deleteWaypoints = 0;
  const int createWaypoints = 1;
  const int continueCurrentWaypoint = 2;

  /*
     * Member variables
     */


  bool obstacleInterrupt;
  bool obstacleDetected;
  bool obstacleAvoided;
  bool clearWaypoints = false;

  float left = 0;
  float center = 0;
  float right = 0;

  int countLeft;
  int countRight;
  bool centerSeen;

  bool ignoreCenter = false;

  Point currentLocation;
  Point driveLocation;
  Point centerLocation;

  bool checkingDriveLocation = false;
  float radius = 1.00;
  int waypointState = -1;

  long int current_time;
  long int timeSinceTags;

  bool targetHeld = false;
  bool previousTargetState = false;


};

#endif // OBSTACLECONTOLLER_H
