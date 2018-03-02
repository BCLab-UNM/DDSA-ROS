#include "SearchController.h"
#include <angles/angles.h>

using namespace std;

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;
  cout << "SearchController -> 0" << endl;

  result.type = waypoint;
result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
  cout << "SearchController -> 1" << endl;

}
	
void SearchController::SetArenaSize(int size)
{
	arena_size = size;
}
	
/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() 
{
  int searchState;
  cout << "TestStatus: SearchController DoWork()" << endl;

  if(!init)
  {
      init = true;
      spiralLocation.x = centerLocation.x;
      spiralLocation.y = centerLocation.y + spacing * CalculateSides(0, 0);
      searchLocation.x = spiralLocation.x;
      searchLocation.y = spiralLocation.y;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
      //cout << "tag: spiral point at corner No. " << cornerNum <<" :" << spiralLocation.x << " , "<< spiralLocation.y << " centerLocation.y : " << centerLocation.y << endl;
      return result;
  }
  else 
  {

    ReachedCheckPoint();
    ReachedSearchLocation();

    if (succesfullPickup) 
    {
      searchState = INSERT_CHECKPOINT;
      cout << "TestStatus: SearchController -> 3" << endl;
    }
    else if (checkpointReached) 
    {
      searchState = TARGET_CURRENTCORNER;
      cout << "TestStatus: SearchController -> checkpoint reached..." << endl;

      if(searchlocationReached)
      {
        searchState = TARGET_NEWCORNER;
        cout << "SearchController -> 5" << endl;
      }
    }
  }
  switch(searchState)
  {
    case INSERT_CHECKPOINT:
    {
      cout << "TestStatus: SearchController -> insert checkpoint" << endl;
      succesfullPickup = false;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.end(), checkPoint);
      return result;
      break;
    }
    case TARGET_CURRENTCORNER:
    {
      cout << "SearchController -> 7" << endl;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.end(), searchLocation);
      return result;
      break;
    }
    case TARGET_NEWCORNER:
    {
      cout << "SearchController -> 8" << endl;
      searchlocationReached = false;
      result.wpts.waypoints.clear();
      searchLocation = SpiralSearching();
      return result;
      break;
    }
  }
}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  if (!result.wpts.waypoints.empty())
  {
	  //cout<<"TestStatus: SearchCTRL waypoint reset:["<<result.wpts.waypoints.back().x<<", "<<result.wpts.waypoints.back().y<<"]"<<endl;
  
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
   }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();


  return false;
}


bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
  if(checkpointReached){
    SetCheckPoint();
    checkpointReached =false;

  }


}

Point SearchController::SpiralSearching(){
  cornerNum +=1;
  if(cornerNum == 4){
    cornerNum = 0;
    stepsIntoSpiral += 1;
  }
  cout << "SearchController -> 13" << endl;
  sideLength = spacing * CalculateSides(stepsIntoSpiral, cornerNum);
  spiralLocation.x = spiralLocation.x + (sideLength * cos(corner));
  spiralLocation.y = spiralLocation.y + (sideLength * sin(corner));
  //cout << "tag: spiral point at corner No. " << cornerNum<<" :" << spiralLocation.x << " , "<< spiralLocation.y << endl;
  //cout << "tag: steps into spiral: " << stepsIntoSpiral << endl;
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
  corner -= (M_PI/2);
  if (corner <= 0.0) {
    corner += 2*M_PI;
  }


  return spiralLocation;


}

void SearchController::SetCheckPoint(){
  // or set it to current location
  cout << "SearchController -> 14" << endl;

  this->checkPoint = this->currentLocation;
  cout << "tag: locating which side of the spiral am I" << endl;



  if(cornerNum == 0){
    cout << "tag: West side of the square " << endl;
    this->checkPoint.y -= 1.0;
    this->checkPoint.x = searchLocation.x;

  }else if(cornerNum == 1){
    cout << "tag: North side of the square" << endl;
    this->checkPoint.x -= 1.0;
    this->checkPoint.y = searchLocation.y;

  }else if(cornerNum == 2){
    cout << "tag: East side of the square" << endl;
    this->checkPoint.y += 1.0;
    this->checkPoint.x = searchLocation.x;

  }else if(cornerNum == 3){
    cout << "tag: South side of the square" << endl;
    this->checkPoint.x += 1.0;
    this->checkPoint.y = searchLocation.y;
  }
  //cout << "tag: CHECKPOINT LOCATION: "<< checkPoint.x << " , "<< checkPoint.y << endl;

}

void SearchController::ReachedCheckPoint(){
  cout << "SearchController -> 15" << endl;

  if (hypot(checkPoint.x-currentLocation.x, checkPoint.y-currentLocation.y) < 0.15) {
    checkpointReached = true;
    cout << "tag: reached the checkpoint(): "<< checkPoint.x<< " , "<< checkPoint.y<< endl;
  }
  else if(!reachedFirstCorner)
  {
    checkpointReached = true;
  }

}

void SearchController::ReachedSearchLocation(){
  cout << "SearchController -> 16" << endl;

  if (hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y) < 0.15) {
    searchlocationReached = true;
    reachedFirstCorner = true;
    cout << "tag: reached the Searchlocation(): " << searchLocation.x<< " , "<< searchLocation.y<< endl;
  }

}

void SearchController::SetRoverIndex(size_t idx){
  roverID = idx;
  cout << "tag:"<< "RoverIndex: "<< roverID << endl;
}

void SearchController::SetSwarmSize(size_t size){
  swarmSize = size;
  cout << "tag:"<< "SwarmSize: "<< swarmSize << endl;
}

float SearchController::CalculateSides( int circuitNum, int slot){
  cout << "SearchController -> 17" << endl;

  constexpr double initial_spiral_offset = 2;

  // North and East
  if(slot == 0 || slot == 1){
    if(circuitNum == 0){
      return roverID + initial_spiral_offset;
    }
    else if(circuitNum == 1){
      sideLength = CalculateSides(0,slot) + swarmSize + roverID + initial_spiral_offset;
      return sideLength;
    }
    else if(circuitNum > 1){
      sideLength = CalculateSides(circuitNum - 1, slot) + 2 * swarmSize;
      return sideLength;
    }
    // South and West
  }else if(slot == 2 || slot == 3){
    if(circuitNum == 0){
      sideLength = CalculateSides(0, 0) + roverID + initial_spiral_offset;
      return sideLength;
    }
    else{
      sideLength = CalculateSides(circuitNum, 0) + swarmSize;
      return sideLength;
    }

  }
  cout << "SearchController -> 18" << endl;


}	

void SearchController::SetReachedWaypoint(bool reached)
{
	reachedWaypoint = reached;
	}
  
	
bool SearchController::OutOfArena(Point location)
{
	double lower = -arena_size/2.0;
	double upper = arena_size/2.0;
	if(location.x -0.5 <= lower || location.y -0.5 <= lower || location.x +0.5 >= upper || location.y +0.5 >= upper)
	{
		return true;
    }
	return false;
}
void SearchController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

Point SearchController::GetCurrentLocation(){
	return this->currentLocation;
	}
