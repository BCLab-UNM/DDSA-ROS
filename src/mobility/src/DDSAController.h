#include <vector> // For pattern vector

#include <string>

struct GoalState
{
  char dir; // Temp for debugging
  float x;
  float y;
  float yaw;
};

class DDSAController
{
  public: 

  DDSAController(float);
  DDSAController(int num_circuits, int num_robots, int robot_index, float gap);

  void setCollectionPoint(float x, float y);
  void generatePattern(int num_circuits, int num_robots, int robot_index);
  GoalState calcNextGoalState(); // Get the next target location
  void setX(float x);
  void setY(float y);
  std::string getPath();
 
  ~DDSAController();
  
 private:

  GoalState getCollectionPointTarget();
  GoalState getTargetN();
  GoalState getTargetS();
  GoalState getTargetE();
  GoalState getTargetW();
  void printPattern();
  int  calcDistanceTravel (int i_robot, int i_circuit, int N_robots, char dir);
  void reversePattern(std::string ith_pattern);  
  int  roverNameToIndex(std::string roverName);
  void setHeadingToNest();

  float x;
  float y;
  float collection_point_x;
  float collection_point_y;
  float step_length;
  std::vector<char> pattern;
  bool initialized;
};
