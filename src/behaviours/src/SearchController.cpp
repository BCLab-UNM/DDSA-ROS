#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  spiralLocation.x = - 1.0;
  spiralLocation.y = 2.0;
  searchLocation.x = - 1.0;
  searchLocation.y = 2.0;
  result.type = waypoint;

}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
  int searchState;

  if(!init){

    init = true;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
    cout << "tag: spiral point at corner No. " << cornerNum<<" :" << spiralLocation.x << " , "<< spiralLocation.y << endl;
    return result;
  }
  else {

    reachedCheckPoint();
    reachedSearchLocation();

    if(succesfullPickup){
      searchState = INSERT_CHECKPOINT;
    }

    else if (checkpointReached) {
      searchState = TARGET_CURRENTCORNER;
      if(searchlocationReached){
        searchState = TARGET_NEWCORNER;
      }

    }
  }

  switch(searchState){
  case INSERT_CHECKPOINT:{
    succesfullPickup = false;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.end(), checkPoint);
    return result;
    break;

  }
  case TARGET_CURRENTCORNER:{
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.end(), searchLocation);
    return result;
    break;

  }
  case TARGET_NEWCORNER:{
    searchlocationReached = false;
    result.wpts.waypoints.clear();
    searchLocation = SpiralSearching();
    return result;
    break;

  }
  }
}

void SearchController::SetCenterLocation(Point centerLocation) {
  //this->centerLocation = centerLocation;
  this->centerLocation.x = centerLocation.x + 1.0;
  this->centerLocation.y = centerLocation.y;
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
    setCheckPoint();
    checkpointReached =false;

  }


}

Point SearchController::SpiralSearching(){
  cornerNum +=1;
  if(cornerNum == 4){
    cornerNum = 0;
  }
  //float corner = 3 * M_PI/4;
  spiralLocation.x = spiralLocation.x + (sideLength * cos(corner));
  spiralLocation.y = spiralLocation.y + (sideLength * sin(corner));
  cout << "tag: spiral point at corner No. " << cornerNum<<" :" << spiralLocation.x << " , "<< spiralLocation.y << endl;
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
  corner -= (M_PI/2);
  if (corner <= 0.0) {
    corner += 2*M_PI;
  }
  sideLength += 0.3;


  return spiralLocation;

  /** for( int it = 0; it <= 3; it++){

    nextLocation.x = centerLocation.x + (sideLength * cos(corner));
    nextLocation.y = centerLocation.y + (sideLength * sin(corner));
    corner -= (M_PI/2);
    sideLength += 0.3;
    cout << "tag: spiral point at :"<< it << " " << nextLocation.x << " , "<< nextLocation.y << endl;
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), nextLocation);

  }  **/


}

void SearchController::setCheckPoint(){
  // or set it to current location
  this->checkPoint = this->currentLocation;
  checkPointExist =true;
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
  cout << "tag: CHECKPOINT LOCATION: "<< checkPoint.x << " , "<< checkPoint.y << endl;

}

void SearchController::reachedCheckPoint(){
  if (hypot(checkPoint.x-currentLocation.x, checkPoint.y-currentLocation.y) < 0.10) {
    checkpointReached = true;
    cout << "tag: reached the checkpoint(): "<< checkPoint.x<< " , "<< checkPoint.y<< endl;
  }

}

void SearchController::reachedSearchLocation(){
  if (hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y) < 0.10) {
    searchlocationReached = true;
    cout << "tag: reached the Searchlocation(): " << searchLocation.x<< " , "<< searchLocation.y<< endl;
  }

}







