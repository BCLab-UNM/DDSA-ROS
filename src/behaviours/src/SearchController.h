#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include <vector>
#include <iostream>
#include "Controller.h"
#include <ros/console.h>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif

#define INSERT_SITEFIDELITY 0
#define TARGET_CURRENTCORNER 1
#define TARGET_NEWCORNER 2
#define ATTEMPT_MAX 8




/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  //CPFAState GetCPFAState() override;
  //void SetCPFAState(CPFAState state) override;
  
  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  CPFAState GetCPFAState() override;
  void SetCPFAState(CPFAState state) override;
  //bool OutOfArena(Point location);
  void SetCurrentLocation(Point currentLocation);
  Point GetCurrentLocation(); //qilu 12/2017
  
  void SetCenterLocation(Point centerLocation);
  void setObstacleAvoidance(bool turn_direction);
  void SetSuccesfullPickup();
  void SetCurrentTimeInMilliSecs( long int time );
  Point SpiralSearching();
  void SetSiteFidelity();
  void setSearchType(bool informed_search);
  void SetArenaSize(int size);
  bool ReachedWaypoint();
  void SetReachedWaypoint(bool reached);
  void ReachedSiteFidelity();
  void ReachedSearchLocation();
  void SetSwarmSize(size_t size);
  void SetRoverIndex(size_t idx);
  bool CreatedFirstWayPoint();  
  float CalculateSides(int circuitNum, int slot);
protected:

  void ProcessData();

private:

  CPFAState cpfa_state = start_state;
  int arena_size = 14.0;
  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  Point siteFidelity;
  Point spiralLocation;
  int attemptCount = 0;
  float sideLength = 1.5;
  //struct for returning data to ROS adapter
  Result result;
  
   
  // Search state
  // Flag to allow special behaviour for the first waypoint
  long int current_time = 0;
  bool succesfullPickup = false;
  int cornerNum = 0;
  float corner = 2 * M_PI;
  bool siteFidelityReached = true;
  bool searchlocationReached = false;
  bool init = false;
  size_t roverID = 0;
  size_t swarmSize = 0;
  int stepsIntoSpiral = 0;
  const float spacing = 0.41;
  bool reachedWaypoint = false;
  bool firstWayPointCreated = false; 
};

#endif /* SEARCH_CONTROLLER */
