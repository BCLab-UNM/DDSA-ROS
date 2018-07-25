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

  result.type = waypoint;
result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
  //attemptCount = 0;
  result.wpts.waypoints.clear();
  succesfullPickup = false;
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
  //cout << "DropTest: SearchController DoWork()" << endl;

  if(!init)
  {
      init = true;
      spiralLocation.x = centerLocation.x;
      spiralLocation.y = centerLocation.y + spacing * CalculateSides(0, 0);
      searchLocation.x = spiralLocation.x;
      searchLocation.y = spiralLocation.y;
      //cout<<"TestStatus: center=["<<centerLocation.x<<", "<<centerLocation.y<<"]"<<endl;
      //cout<<"TestStatus: spiral=["<<spiralLocation.x<<", "<<spiralLocation.y<<"]"<<endl;
      firstWayPointCreated = true;
      
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
      //cout << "tag: spiral point at corner No. " << cornerNum <<" :" << spiralLocation.x << " , "<< spiralLocation.y << " centerLocation.y : " << centerLocation.y << endl;
      return result;
  }
  else 
  {

    ReachedSiteFidelity();
    ReachedSearchLocation();
    
    if (succesfullPickup) 
    {
      searchState = INSERT_SITEFIDELITY;
      //cout << "TestStatus: insert site fidelity..." << endl;
    }
    else if (siteFidelityReached) 
    {
      searchState = TARGET_CURRENTCORNER;
     // cout << "TestStatus: siteFidelity reached, current corner..." << endl;
    }
    //cout<<"giveupStatus: attemptCount="<<attemptCount<<endl;
     if(GetCPFAState() == avoid_obstacle)
      {
          if(attemptCount<8)
	      {
	          attemptCount++;//count the times to approach the location. If the rover always see an obstacle, it should give up. 
		      //cout<<"giveupStatus: travel to the previous location before avoiding obstacles "<<attemptCount<<endl; 
		      //SetCPFAState(reach_search_site);
	      }
	      else
	     {
		      //cout<<"giveupStatus: Give up to previous location *******"<<endl;
              searchlocationReached = true;
              attemptCount = 0;
	      }      	  	
      }
  
  
      if(searchlocationReached)
      {
        searchState = TARGET_NEWCORNER;
        attemptCount = 0;
        siteFidelity.x == 0;
        siteFidelity.y == 0;
        
        //cout << "TestStatus: search location reached, new corner" << endl;
      }
      
      
  }
  
        
        
  switch(searchState)
  {
    case INSERT_SITEFIDELITY:
    {
     //cout << "TestStatus: insert siteFidelity" << endl;
      succesfullPickup = false;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.end(), siteFidelity);
      return result;
      break;
    }
    case TARGET_CURRENTCORNER:
    {
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.end(), searchLocation);
      return result;
      break;
    }
    case TARGET_NEWCORNER:
    {
		//cout<<"TestStatus: target new corner..."<<endl;
      searchlocationReached = false;
      result.wpts.waypoints.clear();
      searchLocation = SpiralSearching();
      return result;
      break;
    }
  }
}

bool SearchController::CreatedFirstWayPoint()
{
	return firstWayPointCreated;
	
	}

CPFAState SearchController::GetCPFAState() 
{
  return cpfa_state;
}

void SearchController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
 //cout<<"SearchCtrl: SetCPFAState ="<<result.cpfa_state <<endl;
}
void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  if (!result.wpts.waypoints.empty())
  {
	//  cout<<"SpiralTest: before waypoint reset:["<<result.wpts.waypoints.back().x<<", "<<result.wpts.waypoints.back().y<<"]"<<endl;
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
  if(siteFidelityReached){
	  cout<<"TestStatusC: set check point after pickup...&&&&&&"<<endl;
    SetSiteFidelity();
    siteFidelityReached =false;

  }


}

Point SearchController::SpiralSearching(){
  cornerNum +=1;
  if(cornerNum == 4){
    cornerNum = 0;
    stepsIntoSpiral += 1;
  }
  //cout << "SearchController -> 13" << endl;
  //cout<<"SpiralTest: corner="<<corner<<endl;
  sideLength = spacing * CalculateSides(stepsIntoSpiral, cornerNum);
  spiralLocation.x += (sideLength * cos(corner));
  spiralLocation.y += (sideLength * sin(corner));
  //cout<<"TestStatus: next spiral=["<<spiralLocation.x<<", "<<spiralLocation.y<<"]"<<endl;
  //cout << "tag: spiral point at corner No. " << cornerNum<<" :" << spiralLocation.x << " , "<< spiralLocation.y << endl;
  //cout << "tag: steps into spiral: " << stepsIntoSpiral << endl;
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
  corner -= (M_PI/2);
  if (corner <= 0.0) {
    corner += 2*M_PI;
  }


  return spiralLocation;


}

void SearchController::SetSiteFidelity(){
  // or set it to current location
  //cout << "SearchController -> 14" << endl;

  this->siteFidelity = this->currentLocation;
  //cout << "tag: locating which side of the spiral am I" << endl;
  //no need to return 1 meter behind on the spiral path  where the rover picked a cube. 
  /*if(cornerNum == 0){
    cout << "SpiralTest:tag: West side of the square " << endl;
    this->checkPoint.y -= 1.0;
    this->checkPoint.x = searchLocation.x;

  }else if(cornerNum == 1){
    cout << "SpiralTest:tag: North side of the square" << endl;
    this->checkPoint.x -= 1.0;
    this->checkPoint.y = searchLocation.y;

  }else if(cornerNum == 2){
    cout << "SpiralTest:tag: East side of the square" << endl;
    this->checkPoint.y += 1.0;
    this->checkPoint.x = searchLocation.x;

  }else if(cornerNum == 3){
    cout << "SpiralTest:tag: South side of the square" << endl;
    this->checkPoint.x += 1.0;
    this->checkPoint.y = searchLocation.y;
  }
  cout << "SpiralTest: tag: CHECKPOINT LOCATION: "<< siteFidelity.x << " , "<< siteFidelity.y << endl;
  */
}

void SearchController::ReachedSiteFidelity(){
  if (siteFidelity.x == 0 && siteFidelity.y == 0){
	  siteFidelityReached = true;  
	  }
  else if (hypot(siteFidelity.x-currentLocation.x, siteFidelity.y-currentLocation.y) < 0.15) {
    siteFidelityReached = true;
    siteFidelity.x = 0;
    siteFidelity.y = 0;
  }
  else{
	  siteFidelityReached = false;
  }
  
}

void SearchController::ReachedSearchLocation(){
  if (hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y) < 0.15) {
    searchlocationReached = true;
  }

}

void SearchController::SetRoverIndex(size_t idx){
  roverID = idx;
  //cout << "tag:"<< "RoverIndex: "<< roverID << endl;
}

void SearchController::SetSwarmSize(size_t size){
  swarmSize = size;
  //cout << "tag:"<< "SwarmSize: "<< swarmSize << endl;
}

float SearchController::CalculateSides( int circuitNum, int slot){
  //cout << "SpiralTest: CalculateSides..." << endl;

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

}	

void SearchController::SetReachedWaypoint(bool reached)
{
	reachedWaypoint = reached;
	}
  
	
/*bool SearchController::OutOfArena(Point location)
{
	double lower = -arena_size/2.0;
	double upper = arena_size/2.0;
	if(location.x -0.5 <= lower || location.y -0.5 <= lower || location.x +0.5 >= upper || location.y +0.5 >= upper)
	{
		return true;
    }
	return false;
}*/
void SearchController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

Point SearchController::GetCurrentLocation(){
	return this->currentLocation;
	}
