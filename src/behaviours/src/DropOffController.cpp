#include "DropOffController.h"

using namespace std;

DropOffController::DropOffController() {

  reachedCollectionPoint = false;
  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;

 // circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;
  pitches = 0.0;

  isPrecisionDriving = false;
  startWaypoint = false;
  timerTimeElapsed = -1;
 
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0; 
}

DropOffController::~DropOffController() {

}

double DropOffController::getPoissonCDF(const double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;
   //cout <<"lambda="<<lambda<<endl;
   //cout <<"get Poisson CDF: local_resource_density="<<local_resource_density<<endl;
  for (size_t i = 1; i <= local_resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }

  return (exp(-lambda) * sumAccumulator);
}

Result DropOffController::DoWork() {

  cout << "DropOffController::DoWork() " << endl;

  int count = countLeft + countRight;

  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
  //to resart our search.
  if(reachedCollectionPoint)
  {
    if (timerTimeElapsed >= 5)
    {
      if (finalInterrupt)
      {
	    //result.lay_pheromone = true; 
        result.type = behavior;
        result.b = nextProcess;		    
        //result.b = COMPLETED;
        result.reset = true;
        //informed_search = true;
        //result.wpts.waypoints.clear();
        //cout<<"Set site fidelity to be the waypoint..."<<endl;
        //result.wpts.waypoints.insert(result.wpts.waypoints.begin(), site_fidelity_location);
        targetHeld = false; //qilu 02/2018
        return result;       
      }
      else
      {
        finalInterrupt = true;
        //cout << "finalInterrupt, true" << endl;
      }
    }
    else if (timerTimeElapsed >= 0.1)
    {
      isPrecisionDriving = true;
      result.type = precisionDriving;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      result.pd.cmdVel = -0.3;
      result.pd.cmdAngularError = 0.0;
    }

    return result;
  }

  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);

  //check to see if we are driving to the center location or if we need to drive in a circle and look.
  if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (count == 0)) {
    //cout<<"distanceToCenter="<<distanceToCenter<<endl;
    result.type = waypoint;
    result.wpts.waypoints.clear();
    //cout<<"TestStatus: dropoffCTRL centerLocation=["<<this->centerLocation.x<<", "<<this->centerLocation.y<<"]"<<endl;
 
    result.wpts.waypoints.push_back(this->centerLocation);
    startWaypoint = false;
    isPrecisionDriving = false;

    timerTimeElapsed = 0;

    return result;

  }
  else if (timerTimeElapsed >= 2)//spin search for center
  {
    Point nextSpinPoint;
    cout<<"TestStatus: spin search for center..."<<endl;
    //sets a goal that is 60cm from the centerLocation and spinner
    //radians counterclockwise from being purly along the x-axis.
    nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
    nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
    nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(nextSpinPoint);

    spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
    if (spinner > 2*M_PI) {
      spinner -= 2*M_PI;
    }
    spinSizeIncrease += spinSizeIncrement/8;
    circularCenterSearching = true;
    //safety flag to prevent us trying to drive back to the
    //center since we have a block with us and the above point is
    //greater than collectionPointVisualDistance from the center.

    returnTimer = current_time;
    timerTimeElapsed = 0;

  }
  bool left = (countLeft > 0);
  bool right = (countRight > 0);
  bool centerSeen = (right || left);
  //reset lastCenterTagThresholdTime timout timer to current time
  if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

    lastCenterTagThresholdTime = current_time;

  }

  if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
  {

    //cout << "CPFAStatus: drive to center" << endl;
    centerSeen = true;

    if (first_center && isPrecisionDriving)
    {
      first_center = false;
      result.type = behavior;
      result.reset = false;
      result.b = nextProcess;
      return result;
    }
    isPrecisionDriving = true;
    if (seenEnoughCenterTags) //if we have seen enough tags
    {
	  if (pitches < -0.5) //turn to the left
      {
		left = true;  
        right = false; 
        }
      else if (pitches > 0.5)//turn to the right
      {
        left = false;
        right = true;
        }
    }
    else //not seen enough tags, then drive forward
    {
		left = false;
		right = false;
		}

    float turnDirection = 1;
    
    //reverse tag rejection when we have seen enough tags that we are on a
    //trajectory in to the square we dont want to follow an edge.
    if (seenEnoughCenterTags) turnDirection = -3;

    result.type = precisionDriving;
    
    //otherwise turn till tags on both sides of image then drive straight
    if (left && right) 
    {
	  result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }
    else if (right) 
    {
	  result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = centeringTurnRate*turnDirection;
    }
    else if (left)
    {
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = -centeringTurnRate*turnDirection;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
      }

    //must see greater than this many tags before assuming we are driving into the center and not along an edge.
    if (count > centerTagThreshold)
    {
      seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
      lastCenterTagThresholdTime = current_time;
    }
    if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
    {
      lastCenterTagThresholdTime = current_time;
    }
    //time since we dropped below countGuard tags
    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

    //we have driven far enough forward to have passed over the circle.
    //cout<<"DropStatus: count="<<count<<endl;
    //cout<<"DropStatus: seenEnoughCenterTags="<<seenEnoughCenterTags<<endl;
    //cout<<"DropStatus: timeSinceSeeingEnoughCenterTags="<<timeSinceSeeingEnoughCenterTags<<endl;
    if (count < 5 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
      centerSeen = false;
    }
    centerApproach = true;
    prevCount = count;
    count = 0;
    countLeft = 0;
    countRight = 0;
  }

  //was on approach to center and did not seenEnoughCenterTags
  //for lostCenterCutoff seconds so reset.
  else if (centerApproach) {
    //cout<<"was on approach to center and did not seenEnoughCenterTags"<<endl;
    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
    if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
    {
      //cout << "back to drive to center base location..." << endl;
      //go back to drive to center base location instead of drop off attempt
      reachedCollectionPoint = false;
      seenEnoughCenterTags = false;
      centerApproach = false;

      result.type = waypoint;
      //cout<<"wpts.waypoint to center"<<endl;
      result.wpts.waypoints.push_back(this->centerLocation);
      if (isPrecisionDriving) {
        result.type = behavior;
        result.b = prevProcess;
        result.reset = false;
      }
      isPrecisionDriving = false;
      interrupt = false;
      precisionInterrupt = false;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    return result;

  }

  if (!centerSeen && seenEnoughCenterTags)
  {
   //cout<<"not seen center and seen enough tags"<<endl;
    reachedCollectionPoint = true;
    centerApproach = false;
    returnTimer = current_time;
  }

  return result;
}

void DropOffController::Reset() {
	//cout<<"DropOffController::Reset()"<<endl;
  result.type = behavior;
  result.b = wait;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  //result.lay_pheromone = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;
  pitches = 0.0;


  //reset flags
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  circularCenterSearching = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  targetHeld = false;
  startWaypoint = false;
  first_center = true;

}

/*void DropOffController::senseLocalResourceDensity(int num_tags)
{
  //if(num_tags > local_resource_density){
    local_resource_density = num_tags;
  //}

  cout << "Dropoff: local_resource_density: " << local_resource_density << endl;
}*/

void DropOffController::SetTagData(vector<Tag> tags) {
  countRight = 0;
  countLeft = 0;
  pitches = 0.0;
  //yaws = 0.0;

  if(targetHeld) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 && !reachedCollectionPoint) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {

          // checks if tag is on the right or left side of the image
          if (tags[i].getPositionX() + cameraOffsetCorrection > 0) 
          {
            countRight++;
          } 
          else 
          {
            countLeft++;
          }
          pitches += tags[i].calcPitch();
          //yaws += tags[i].calcYaw();
        }
      }
      pitches /= (countLeft + countRight);
      //yaws /= (countLeft + countRight);
    }
  }

}

void DropOffController::ProcessData() {
  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  } else {
    startWaypoint = true;
  }
}

bool DropOffController::ShouldInterrupt() {
	//cout<<"dropoff controller should interrupt..."<<endl;
  ProcessData();
  if (startWaypoint && !interrupt) {
    interrupt = true;
    precisionInterrupt = false;
    //cout<<"D: true d1"<<endl;
    return true;
    
  }
  else if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    //cout<<"D: true d2"<<endl;
    return true;
    
  }
  if (finalInterrupt) {
	  //cout<<"D: true d3"<<endl;
    return true;
  }
  //cout<<"false"<<endl; //do not cout there. this will affact the behavior of robots
}

bool DropOffController::HasWork() {
  
  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
    return false;
  }
   //cout <<"Dropoff has work..."<<(startWaypoint || isPrecisionDriving)<<endl;
  return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
  targetHeld = true;
  //site_fidelity_location = currentLocation;//qilu 11/2017
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
  targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
CPFAState DropOffController::GetCPFAState() 
{
  return cpfa_state;
}

void DropOffController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
}


