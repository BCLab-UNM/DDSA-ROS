#include "LogicController.h"

using namespace std;

LogicController::LogicController() {
 //cout<<"start logic controller..."<<endl;
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;
  rng = new random_numbers::RandomNumberGenerator();
  
  ProcessData();

  control_queue = priority_queue<PrioritizedController>();

}

LogicController::~LogicController() {}

void LogicController::Reset() {

  std::cout << "LogicController.Reset()" << std::endl;
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();
}

//******************************************************************************
// This function is called every 1/10th of a second by the ROSAdapter
// The logical flow if the behaviours is controlled here by using an interrupt,
// haswork, and priority queue system.
Result LogicController::DoWork()
{
  Result result;
  
  cout << "LogicController:DoWork()...Proccess State is " << processState << endl;
  //first a loop runs through all the controllers who have a priority of 0 or above with the largest number being
  // above with the largest number being most important. A priority of less than
  // 0 is an ignored controller (we will use -1 as the standard for an ignored
  // controller). If any controller needs an interrupt, the logic state is
  // changed to interrupt
  for(PrioritizedController cntrlr : prioritizedControllers)
  {
    if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0)
    {
	  cout<<" is interrupt..."<<endl;
      logicState = LOGIC_STATE_INTERRUPT;
      //do not break all shouldInterupts may need calling in order to properly pre-proccess data.
    }
  }

  switch(logicState) {

  // ***************************************************************************
  // BEGIN LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // Enter this state when an interrupt has been thrown or there are no pending
  // control_queue.top().actions.
  case LOGIC_STATE_INTERRUPT: {
    // Reset the control queue
    control_queue = priority_queue<PrioritizedController>();

    //check what controllers have work to do all that say yes will be added to the priority queue.
    for(PrioritizedController cntrlr : prioritizedControllers) {
		cout <<"priority="<<cntrlr.priority<<endl;
		cout<< "name="<<cntrlr.controller <<endl;
      if(cntrlr.controller->HasWork()) {
        if (cntrlr.priority < 0) {
          continue;
        }
        else {
          control_queue.push(cntrlr);
        }
      }
    }

    // If no controlers have work, report this to ROS Adapter and do nothing.
    if(control_queue.empty()) {
      result.type = behavior;
      result.b = wait;
      break;
    }
    else {
      //default result state if someone has work this safe gaurds against faulty result types
      result.b = noChange;
    }

    // Take the top member of the priority queue and run its do work function.
    result = control_queue.top().controller->DoWork();
    
   
	
    // Analyze the result that was returned and do state changes accordingly.
    // Behavior types are used to indicate behavior changes.
   //cout<<"result.type="<<result.type<<endl;
    if(result.type == behavior) {
     //cout<<"result type == behavior"<<endl;
      // Ask for an external reset so the state of the controller is preserved
      // until after it has returned a result and gotten a chance to communicate
      // with other controllers.
      if (result.reset) {
		 //cout<<"result reset..."<<endl;
        controllerInterconnect(); // Allow controller to communicate state data before it is reset.
       //cout<<"controller="<<control_queue.top().controller<<endl;
        control_queue.top().controller->Reset();
      }

      //ask for the procces state to change to the next state or loop around to the begining
      if(result.b == nextProcess) {
		 cout<<"TestStatus: next process..."<<endl;
        if (processState == _LAST - 1) {
          processState = _FIRST;
        }
        else {
          processState = (ProcessState)((int)processState + 1);
        }
      }
      // Ask for the procces state to change to the previouse state or loop around to the end.
      else if(result.b == prevProcess) {
        if (processState == _FIRST) {
          processState = (ProcessState)((int)_LAST - 1);
        }
        else {
          processState = (ProcessState)((int)processState - 1);
        }
      }

      //update the priorites of the controllers based upon the new process state.
      if (result.b == nextProcess || result.b == prevProcess) {
        ProcessData();
        result.b = wait;
        driveController.Reset(); //it is assumed that the drive controller may be in a bad state if interrupted so reset it
      }
      break;
    }

    // Precision driving result types are when a controller wants direct
    // command of the robots actuators. LogicController facilitates the command
    // pass through in the LOGIC_STATE_PRECISION_COMMAND switch case.
    else if(result.type == precisionDriving) {
     cout<<"TestStatus: result type == precisionDriving..."<<endl;
      logicState = LOGIC_STATE_PRECISION_COMMAND;
     //cout<<"logicState="<<logicState<<endl;
      break; 

    }

    // Waypoints are also a pass through facilitated command but with a slightly
    // diffrent overhead. They are handled in the LOGIC_STATE_WAITING switch case.
    else if(result.type == waypoint) {
     //cout<<"result type == waypoint"<<endl;
      logicState = LOGIC_STATE_WAITING;
     //cout<<"logicState ="<<logicState<<endl;
      driveController.SetResultData(result);
      // Fall through on purpose to "case LOGIC_STATE_WAITING:"
    }

  }
  // ***************************************************************************
  // END LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_WAITING
  // ***************************************************************************

  // This case is primarly when logic controller is waiting for drive controller
  // to reach its last waypoint.
  case LOGIC_STATE_WAITING: {
     //cout <<"logic state waiting..."<<LOGIC_STATE_WAITING<<endl;
    //cout<<"logicState="<<logicState<<endl; 
    // Ask drive controller how to drive: specifically, return commands to be
    // passed to the ROS Adapter such as left and right wheel PWM values in the
    // result struct.
    result = driveController.DoWork();
    //cout<<"CPFAStatus: LogicController: result.wpts.waypoint=["<<result.wpts.waypoints[0].x<<" size="<<result.wpts.waypoints.size()<<endl;
    // When out of waypoints, the drive controller will throw an interrupt.
    // However, unlike other controllers, drive controller is not on the
    // priority queue so it must be checked here.
    if (result.type == behavior) 
    {
		cout <<"logic state== waiting; result type == behavior"<<endl;
		if(driveController.ShouldInterrupt()) 
        {
          logicState = LOGIC_STATE_INTERRUPT;
         //cout<<"SwitchStatus: driver controller interrupt...true"<<endl;
          if(processState == PROCCESS_STATE_SEARCHING)
          {
		   cout<<"TestStatusSwitchStatus: searchCtrl set reached..."<<endl;
		    searchController.SetReachedWaypoint(true);
		    
		  } 
      }
    }
    break;
  }
  // ***************************************************************************
  // END LOGIC_STATE_WAITING
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_PRECISION_COMMAND
  // ***************************************************************************

    // Used for precision driving pass through.
  case LOGIC_STATE_PRECISION_COMMAND: {
   //cout<<"logic state precision command..."<<endl;
    // Unlike waypoints, precision commands change every update tick, so we ask
    // the controller for new commands on every update tick.
   //cout<<"control_queue.top().controller="<<control_queue.top().controller<<endl;
    result = control_queue.top().controller->DoWork();

    // Pass the driving commands to the drive controller so it can interpret them.
    driveController.SetResultData(result);

    // The interpreted commands are turned into proper initial_spiral_offset
    // motor commands to be passed the ROS Adapter such as left and right wheel
    // PWM values in the result struct.
    result = driveController.DoWork();
    break;

  }
  // ***************************************************************************
  // END LOGIC_STATE_PRECISION_COMMAND
  // ***************************************************************************
}

   // bad! causes node to crash


  //now using proccess logic allow the controller to communicate data between eachother
  controllerInterconnect();

  // Give the ROSAdapter the final decision on how it should drive.
  return result;
}

void LogicController::UpdateData()
{
  // As the top level controller, there is no specific data that must
  // be updated in this controller. This function may be of greater use
  // and importance in lower level controllers.
}

void LogicController::ProcessData() 
{

  //this controller priority is used when searching
  if (processState == PROCCESS_STATE_SEARCHING) 
  {
	  cout<<"PROCCESS_STATE_SEARCHING ..."<<endl;
	prioritizedControllers = {
	PrioritizedController{15, (Controller*)(&pickUpController)},
	PrioritizedController{10, (Controller*)(&obstacleController)},
	PrioritizedController{5, (Controller*)(&rangeController)},
	PrioritizedController{1, (Controller*)(&searchController)},
	PrioritizedController{-1, (Controller*)(&dropOffController)},
	PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }
  // This priority is used when returning a target to the center collection zone.
  else if (processState  == PROCCESS_STATE_TARGET_PICKEDUP)
  {
	  cout<<"PROCCESS_STATE_TARGET_PICKEDUP ..."<<endl;
    prioritizedControllers = {
    PrioritizedController{15, (Controller*)(&obstacleController)},
    PrioritizedController{10, (Controller*)(&rangeController)},
	PrioritizedController{-1, (Controller*)(&pickUpController)},
    PrioritizedController{-1, (Controller*)(&searchController)},
    
    
    PrioritizedController{1, (Controller*)(&dropOffController)},
    PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }
  //this priority is used when returning a target to the center collection zone
  else if (processState  == PROCCESS_STATE_DROP_OFF)
  {
    prioritizedControllers = {
      PrioritizedController{10, (Controller*)(&rangeController)},
      PrioritizedController{1, (Controller*)(&dropOffController)},
PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
            PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }

  // Under manual control ONLY the manual waypoint controller is active.
  else if (processState == PROCCESS_STATE_MANUAL) {
    // under manual control only the manual waypoint controller is active
    prioritizedControllers = {
      PrioritizedController{5,  (Controller*)(&manualWaypointController)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&rangeController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
}

bool LogicController::ShouldInterrupt()
{
	//cout<<"logic controller should not interrupt..."<<endl;
  ProcessData();

  // The logic controller is the top level controller and will never have to
  // interrupt. It is only the lower level controllers that may need to interupt.
  return false;
}

bool LogicController::HasWork()
{
  // do because it is always handling the work of the other controllers.
  return false;
}

int LogicController::getCollisionCalls()
{
	if(obstacleController.HasWork())
	{
		cout<<"ObstacleState: get one obstacle avoidance call..."<<endl;
		return 1;
		}
		
	return 0;
}

// This function will deal with inter-controller communication. Communication
// that needs to occur between specific low level controllers is done here.
//
// The type of communication may or may not depend on the processState.
//
//                       /<----> ControllerA
// LogicController <---->|                  \__ inter-controller communication
//                       |                  /
//                       \<----> ControllerB
void LogicController::controllerInterconnect()
{
 cout<<"controller interconnect..."<<endl;
 cout<<"processState="<<processState<<endl;
  if (processState == PROCCESS_STATE_SEARCHING)
  {
   //cout<<"state searching" <<endl;
    // Obstacle controller needs to know if the center ultrasound should be ignored.
    if(pickUpController.GetIgnoreCenter())
    {
      obstacleController.SetIgnoreCenterSonar();
    }

    //pickup controller annouces it has pickedup a target
    if(pickUpController.GetTargetHeld()) 
    {
	//	cout<<"get target held..."<<endl;
      dropOffController.SetTargetPickedUp();
      obstacleController.SetTargetHeld();
      searchController.SetSuccesfullPickup();
    }
  }

  //ask if drop off has released the target from the claws yet
  if (!dropOffController.HasTarget()) 
  {
    obstacleController.SetTargetHeldClear();
  }

  // Obstacle controller is running and driveController needs to clear its waypoints.
  if(obstacleController.GetShouldClearWaypoints())
  {
	 //cout<<"clear waypoints..."<<endl;
    driveController.Reset();
  }

}

void LogicController::SetArenaSize(int size)
{
	cout<<"set arena size "<< size<<endl;
	searchController.SetArenaSize(size);
	}

// Receives position in the world inertial frame (should rename to SetOdomPositionData).
void LogicController::SetPositionData(Point currentLocation)
{
	//cout<<"TestStatus: logicCTRL set current position=["<< currentLocation.x<<","<<currentLocation.y<<"]"<<endl;
  searchController.SetCurrentLocation(currentLocation);
  dropOffController.SetCurrentLocation(currentLocation);
  obstacleController.SetCurrentLocation(currentLocation);
  driveController.SetCurrentLocation(currentLocation);
  manualWaypointController.SetCurrentLocation(currentLocation);
}


// Recieves position in the world frame with global data (GPS).
void LogicController::SetMapPositionData(Point currentLocation)
{
  rangeController.SetCurrentLocation(currentLocation);  
}

// Sets the velocity data for the driveController. This information is
// necessary so that the drive controller can update the velocity sent
// back to the RosAdapter correctly.
void LogicController::SetVelocityData(float linearVelocity, float angularVelocity)
{
  driveController.SetVelocityData(linearVelocity,angularVelocity);
}

void LogicController::SetMapVelocityData(float linearVelocity, float angularVelocity)
{
}

// Give the specified controllers a list of visible april tags.
void LogicController::SetAprilTags(vector<Tag> tags)
{
  pickUpController.SetTagData(tags);
  obstacleController.SetTagData(tags);
  dropOffController.SetTagData(tags);
}

// Give the specified controllers the sonar sensor values.
void LogicController::SetSonarData(float left, float center, float right)
{
  // The pickUpController only needs the center data in order to tell if
  // an april tag cube has been picked up correctly.
  pickUpController.SetSonarData(center);

  obstacleController.SetSonarData(left,center,right);
}

// Called once by RosAdapter in guarded init.
void LogicController::SetCenterLocationOdom(Point centerLocationOdom)
{
  searchController.SetCenterLocation(centerLocationOdom);
  dropOffController.SetCenterLocation(centerLocationOdom);
  //pheromoneController.SetCenterLocation(centerLocationOdom);
}

void LogicController::SetRoverInitLocation(Point location) //for mapping the locations of pheromone trails to other rovers.
{
  dropOffController.SetRoverInitLocation(location);
}

void LogicController::AddManualWaypoint(Point manualWaypoint, int waypoint_id)
{
  manualWaypointController.AddManualWaypoint(manualWaypoint, waypoint_id);
}

void LogicController::RemoveManualWaypoint(int waypoint_id)
{
  manualWaypointController.RemoveManualWaypoint(waypoint_id);
}

std::vector<int> LogicController::GetClearedWaypoints()
{
  return manualWaypointController.ReachedWaypoints();
}

void LogicController::setVirtualFenceOn( RangeShape* range )
{
  rangeController.SetRangeShape(range);
  rangeController.SetEnabled(true);
}

void LogicController::setVirtualFenceOff()
{
  rangeController.SetEnabled(false);
}

void LogicController::SetCenterLocationMap(Point centerLocationMap) 
{
}

void LogicController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
  dropOffController.SetCurrentTimeInMilliSecs( time );
  pickUpController.SetCurrentTimeInMilliSecs( time );
  obstacleController.SetCurrentTimeInMilliSecs( time );
  driveController.SetCurrentTimeInMilliSecs(time);
  searchController.SetCurrentTimeInMilliSecs( time);
}

void LogicController::SetSwarmSize(size_t size) {
  searchController.SetSwarmSize(size);
}
void LogicController::printCPFAState() {
  cout << "CPFAState: ";
  /*if(cpfa_state == set_target_location)
    cout << "set_target_location" << endl;
  else if(cpfa_state == travel_to_search_site)
    cout << "travel_to_search_site" << endl;
  else if(cpfa_state == search_with_uninformed_walk)
    cout << "search_with_uninformed_walk" << endl;
  else if(cpfa_state == search_with_informed_walk)
    cout << "search_with_informed_walk" << endl;
  else if(cpfa_state == sense_local_resource_density)
    cout << "sense_local_resource_density" << endl;
  else if (cpfa_state == return_to_nest)
    cout << "return_to_nest" << endl;
  else
    cout << "WTF" << endl;*/
}

void LogicController::SetRoverIndex(size_t idx) {
  searchController.SetRoverIndex(idx);

}

Point LogicController::GetCurrentLocation() {
  return searchController.GetCurrentLocation();
}

void LogicController::SetCPFAState(CPFAState state) {
  if(state != cpfa_state) {
    cpfa_state = state;
   //cout<<"SwitchStatus: logic, state="<<cpfa_state<<endl;
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(state != cntrlr.controller->GetCPFAState()) {
        cntrlr.controller->SetCPFAState(state);
      }
    }
  }

  printCPFAState();
}

CPFAState LogicController::GetCPFAState() {
  return cpfa_state;
}


void LogicController::SetModeAuto() {
  if(processState == PROCCESS_STATE_MANUAL) {
    // only do something if we are in manual mode
   //cout<<"do something if we are in manual mode..."<<endl;
    this->Reset();
    manualWaypointController.Reset();
  }
}
void LogicController::SetModeManual()
{
  if(processState != PROCCESS_STATE_MANUAL) {
    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCCESS_STATE_MANUAL;
    ProcessData();
    control_queue = priority_queue<PrioritizedController>();
    driveController.Reset();
  }
}

