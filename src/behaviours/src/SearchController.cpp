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
      spiralLocation.x = centerLocation.x;
      spiralLocation.y = centerLocation.y + spacing * CalculateSides(0, 0);
      searchLocation.x = spiralLocation.x;
      searchLocation.y = spiralLocation.y;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
      cout << "tag: spiral point at corner No. " << cornerNum <<" :" << spiralLocation.x << " , "<< spiralLocation.y << endl;
      return result;
  }
  else {

    ReachedCheckPoint();
    ReachedSearchLocation();

    if(succesfullPickup){
      searchState = INSERT_CHECKPOINT;
    }

    else if (checkpointReached) {
      searchState = TARGET_CURRENTCORNER;
      if(searchlocationReached){
        searchState = TARGET_NEWCORNER;
      }else if( hasObstacleDetected && !searchlocationReached){
        searchState = PATHPLANNING_CURRENTCORNER;
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
  case PATHPLANNING_CURRENTCORNER:{
    hasObstacleDetected = false;
    result.wpts.waypoints.clear();
    PathPlanningWaypoints();
    return result;
    break;


  }
  }
}

void SearchController::SetCenterLocation(Point centerLocation) {
  this->centerLocation.x = centerLocation.x;
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

  sideLength = spacing * CalculateSides(stepsIntoSpiral, cornerNum);
  spiralLocation.x = spiralLocation.x + (sideLength * cos(corner));
  spiralLocation.y = spiralLocation.y + (sideLength * sin(corner));
  cout << "tag: spiral point at corner No. " << cornerNum<<" :" << spiralLocation.x << " , "<< spiralLocation.y << endl;
  cout << "tag: steps into spiral: " << stepsIntoSpiral << endl;
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);
  corner -= (M_PI/2);
  if (corner <= 0.0) {
    corner += 2*M_PI;
  }


  return spiralLocation;


}

void SearchController::SetCheckPoint(){
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

void SearchController::ReachedCheckPoint(){
  if (hypot(checkPoint.x-currentLocation.x, checkPoint.y-currentLocation.y) < 0.10) {
    checkpointReached = true;
    cout << "tag: reached the checkpoint(): "<< checkPoint.x<< " , "<< checkPoint.y<< endl;
  }

}

void SearchController::ReachedSearchLocation(){
  if (hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y) < 0.10) {
    searchlocationReached = true;
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

  constexpr double center_distance = 1.308;
  constexpr double initial_spiral_offset = center_distance / 2.0;

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

void SearchController::ObstacleDetected(){
 hasObstacleDetected = true;
}

void SearchController::PathPlanningWaypoints(){
  Point reDirectLocation = currentLocation;

  reDirectLocation.x = reDirectLocation.x + (0.50 * cos(M_PI)); // tried M_PI and M_PI/2
  reDirectLocation.y = reDirectLocation.y + (0.50 * sin(M_PI));

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), reDirectLocation);

  reDirectLocation.x = reDirectLocation.x + (1.00 * cos(M_PI/2));
  reDirectLocation.y = reDirectLocation.y + (1.00 * sin(M_PI/2));

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), reDirectLocation);

  reDirectLocation.x = reDirectLocation.x + (1.00 * cos(0)); // tried M_PI and M_PI/2
  reDirectLocation.y = reDirectLocation.y + (1.00 * sin(0));

  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), reDirectLocation);


}




   // QPointF rover_positions[6] =
   // {
      /* cardinal rovers: North, East, South, West */
      //QPointF(-1.308,  0.000), // 1.308 = distance_from_center_to_edge_of_collection_zone
     // QPointF( 0.000, -1.308), //             + 50 cm distance to rover
     // QPointF( 1.308,  0.000), //             + 30 cm distance_from_center_of_rover_to_edge_of_rover
     // QPointF( 0.000,  1.308), // 1.308m = 0.508m + 0.5m + 0.3m

      /////* corner rovers: Northeast, Southwest */
      //QPointF( 1.072,  1.072), // 1.072 = diagonal_distance_from_center_to_edge_of_collection_zone
      //QPointF(-1.072, -1.072)  //             + diagonal_distance_to_move_50cm
   // };                         //             + diagonal_distance_to_move_30cm
                               // 1.072m = 0.508 + 0.354 + 0.212







