#include "PickUpController.h"
#include <limits> // For numeric limits
#include <cmath> // For hypot

PickUpController::PickUpController() {
  lockTarget = false;
  timeOut = false;
  nTargetsSeen = 0;
  blockYawError = 0;
  blockDistance = 0;
  timeDifference = 0;

  targetFound = false;
 cout << "PickUPController -> 0" << endl;
  result.type = precisionDriving;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError= 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.PIDMode = SLOW_PID;

}

PickUpController::~PickUpController() {
}

void PickUpController::SetTagData(vector<TagPoint> tags) {
cout << "PickUPController -> 1" << endl;
  if (tags.size() > 0) {

    nTargetsSeen = tags.size();

    double closest = std::numeric_limits<double>::max();
    int target  = 0;
    for (int i = 0; i < tags.size(); i++) { //this loop selects the closest visible block to makes goals for it

      if (tags[i].id == 0) {

        targetFound = true;

        double test = hypot(hypot(tags[i].x, tags[i].y), tags[i].z); //absolute distance to block from camera lense
        if (closest > test)
        {
          target = i;
          closest = test;
        }
      }
      else {
        cout << "PickUPController -> 2" << endl;
        nTargetsSeen--;

        if(tags[i].id == 256)
        {
          Reset();
          if (has_controle)
          {
            cout << "pickup reset return interupt free" << endl;
            interupte_release = true;
          }
          return;
        }
      }
    }

    float cameraOffsetCorrection = 0.023; //meters;

    blockYawError = atan((tags[target].x + cameraOffsetCorrection)/blockDistance)*1.05; //angle to block from bottom center of chassis on the horizontal.

    ///TODO: Explain the trig going on here- blockDistance is c, 0.195 is b; find a
    blockDistance = hypot(tags[target].z, tags[target].y); //distance from bottom center of chassis ignoring height.
cout << "PickUPController -> 3" << endl;
    blockDistanceFromCamera = hypot(hypot(tags[target].x, tags[target].y), tags[target].z);
  }

}


bool PickUpController::SetSonarData(float rangeCenter){
cout << "PickUPController -> 4" << endl;
  if (rangeCenter < 0.12 && targetFound) {
    result.type = behavior;
    result.b = nextProcess;
    result.reset = true;
    targetHeld = true;
    return true;
  }

  return false;

}

void PickUpController::ProcessData() {
  if(!targetFound){
    // Do nothing
    cout << "PickUPController -> 5" << endl;
    return;
  }

  if ( (blockDistance*blockDistance - 0.195*0.195) > 0 )
  {
    blockDistance = sqrt(blockDistance*blockDistance - 0.195*0.195);
  }
  else
  {
    float epsilon = 0.00001; // A small non-zero positive number
    blockDistance = epsilon;
    cout << "PickUPController -> 6" << endl;
  }

  //if target is close enough
  //diffrence between current time and millisecond time
  long int Tdiff = current_time - millTimer;
  float Td = Tdiff/1e3;

  if (blockDistanceFromCamera < 0.12 && Td < 2.7) {
cout << "PickUPController -> 7" << endl;
    result.type = behavior;
    result.b = nextProcess;
    result.reset = true;
    targetHeld = true;
  }
  //Lower wrist and open fingures if no locked targt
  else if (!lockTarget)
  {
    //set gripper;
    result.fingerAngle = M_PI_2;
    result.wristAngle = 1.25;
  }
}


bool PickUpController::ShouldInterrupt(){

  ProcessData();

  if (interupte_release)
  {
    cout << "PickUPController -> 8" << endl;
    interupte_release = false;
    has_controle = false;
    return true;
  }

  if ((targetFound && !interupted) || targetHeld) {
    interupted = true;
    has_controle = false;
    return true;
  }
  else if (!targetFound && interupted) {
    interupted = false;
    has_controle = false;
    return true;
  }
  else {
    cout << "PickUPController -> 9" << endl;
    return false;
  }
}

Result PickUpController::DoWork() {
cout << "PickUPController -> 10" << endl;
  has_controle = true;

  if (!targetHeld) {
    //threshold distance to be from the target block before attempting pickup
    float targetDistance = 0.15; //meters

    // millisecond time = current time if not in a counting state
    if (!timeOut) millTimer = current_time;

    //diffrence between current time and millisecond time
    long int Tdifference = current_time - millTimer;
    float Td = Tdifference/1e3;
    timeDifference = Td;

    if (nTargetsSeen == 0 && !lockTarget) //if not targets detected and a target has not been locked in
    {
      if(!timeOut) //if not in a counting state
      {
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngularError= 0.0;
        result.wristAngle = 0.8;

        timeOut = true;
        result.pd.cmdAngularError= -blockYawError;
      }
      //if in a counting state and has been counting for 1 second
      else if (Td > 1 && Td < 2.5)
      {
        result.pd.cmdVel = -0.2;
        result.pd.cmdAngularError= 0.0;
      }
    }
    else if (blockDistance > targetDistance && !lockTarget) //if a target is detected but not locked, and not too close.
    {
      cout << "PickUPController -> 11" << endl;
      float vel = blockDistance * 0.20;
      if (vel < 0.1) vel = 0.1;
      if (vel > 0.2) vel = 0.2;
      result.pd.cmdVel = vel;
      result.pd.cmdAngularError = -blockYawError;
      timeOut = false;
      nTargetsSeen = 0;
      return result;
    }
    else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
    {
      lockTarget = true;
      result.pd.cmdVel = 0.15;
      result.pd.cmdAngularError= 0.0;
      timeOut = true;
      ignoreCenterSonar = true;
    }
    else if (Td > 2.2) //raise the wrist
    {
      result.pd.cmdVel = -0.15;
      result.pd.cmdAngularError= 0.0;
      result.wristAngle = 0;
    }
    else if (Td > 1.7) //close the fingers and stop driving
    {
      result.pd.cmdVel = -0.1;
      result.pd.cmdAngularError= 0.0;
      result.fingerAngle = 0;
      return result;
    }

    if (Td > 3.4 && timeOut) {
      lockTarget = false;
      ignoreCenterSonar = true;
    }
    else if (Td > 2.8 && timeOut) //if enough time has passed enter a recovery state to re-attempt a pickup
    {


      result.pd.cmdVel = -0.15;
      result.pd.cmdAngularError= 0.0;
      //set gripper to open and down
      result.fingerAngle = M_PI_2;
      result.wristAngle = 0;

    }

    if (Td > 4.0 && timeOut) //if no targets are found after too long a period go back to search pattern
    {
      Reset();
      interupted = true;
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngularError= 0.0;
      ignoreCenterSonar = true;
    }
  }

  return result;
}

bool PickUpController::HasWork() {
  return targetFound;
  cout << "PickUPController -> 12" << endl;
}

void PickUpController::Reset() {

  result.type = precisionDriving;
  result.PIDMode = SLOW_PID;
  lockTarget = false;
  timeOut = false;
  nTargetsSeen = 0;
  blockYawError = 0;
  blockDistance = 0;
  timeDifference = 0;

  targetFound = false;
  interupted = false;
  targetHeld = false;

  result.pd.cmdVel = 0;
  result.pd.cmdAngularError= 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.reset = false;

  ignoreCenterSonar = false;
}

void PickUpController::SetUltraSoundData(bool blockBlock){
  this->blockBlock = blockBlock;
}

void PickUpController::SetCurrentTimeInMilliSecs( long int time )
{
  cout << "PickUPController -> 13" << endl;
  current_time = time;
}
