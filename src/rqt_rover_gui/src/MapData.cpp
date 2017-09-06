#include "MapData.h"

using namespace std;

MapData::MapData()
{
  display_global_offset = false;
}

void MapData::addToGPSRoverPath(string rover, float x, float y)
{
  // Negate the y direction to orient the map so up is north.
  y = -y;

  if (x > max_gps_seen_x[rover]) max_gps_seen_x[rover] = x;
  if (y > max_gps_seen_y[rover]) max_gps_seen_y[rover] = y;
  if (x < min_gps_seen_x[rover]) min_gps_seen_x[rover] = x;
  if (y < min_gps_seen_y[rover]) min_gps_seen_y[rover] = y;

  update_mutex.lock();

  // by default, if a rover's global offset is not set, default constructors will do two things here:
  //   1) std::map's default constructor creates a rover_global_offsets for the rover in the map
  //   2) std::pair's default constructor creates a pair for the global offset set to (0.0, 0.0)
  // and finally, after said objects are constructed
  //   3) we retrieve that (0.0, 0.0) pair, resulting in no offset being added to the rover path
  float offset_x = rover_global_offsets[rover].first;
  float offset_y = rover_global_offsets[rover].second;
  global_offset_gps_rover_path[rover].push_back(pair<float,float>(x+offset_x,y-offset_y));

  gps_rover_path[rover].push_back(pair<float,float>(x,y));

  update_mutex.unlock();

}

void MapData::addToEncoderRoverPath(string rover, float x, float y)
{
  // Negate the y direction to orient the map so up is north.
  y = -y;

  if (x > max_encoder_seen_x[rover]) max_encoder_seen_x[rover] = x;
  if (y > max_encoder_seen_y[rover]) max_encoder_seen_y[rover] = y;
  if (x < min_encoder_seen_x[rover]) min_encoder_seen_x[rover] = x;
  if (y < min_encoder_seen_y[rover]) min_encoder_seen_y[rover] = y;

  update_mutex.lock();

  // by default, if a rover's global offset is not set, default constructors will do two things here:
  //   1) std::map's default constructor creates a rover_global_offsets for the rover in the map
  //   2) std::pair's default constructor creates a pair for the global offset set to (0.0, 0.0)
  // and finally, after said objects are constructed
  //   3) we retrieve that (0.0, 0.0) pair, resulting in no offset being added to the rover path
  float offset_x = rover_global_offsets[rover].first;
  float offset_y = rover_global_offsets[rover].second;
  global_offset_encoder_rover_path[rover].push_back(pair<float,float>(x+offset_x,y-offset_y));

  encoder_rover_path[rover].push_back(pair<float,float>(x,y));

  update_mutex.unlock();
}


void MapData::addToEKFRoverPath(string rover, float x, float y)
{
  // Negate the y direction to orient the map so up is north.
  y = -y;

  if (x > max_ekf_seen_x[rover]) max_ekf_seen_x[rover] = x;
  if (y > max_ekf_seen_y[rover]) max_ekf_seen_y[rover] = y;
  if (x < min_ekf_seen_x[rover]) min_ekf_seen_x[rover] = x;
  if (y < min_ekf_seen_y[rover]) min_ekf_seen_y[rover] = y;

  update_mutex.lock();

  // by default, if a rover's global offset is not set, default constructors will do two things here:
  //   1) std::map's default constructor creates a rover_global_offsets for the rover in the map
  //   2) std::pair's default constructor creates a pair for the global offset set to (0.0, 0.0)
  // and finally, after said objects are constructed
  //   3) we retrieve that (0.0, 0.0) pair, resulting in no offset being added to the rover path
  float offset_x = rover_global_offsets[rover].first;
  float offset_y = rover_global_offsets[rover].second;
  global_offset_ekf_rover_path[rover].push_back(pair<float,float>(x+offset_x,y-offset_y));

  ekf_rover_path[rover].push_back(pair<float,float>(x,y));

  update_mutex.unlock();
}

void MapData::addTargetLocation(string rover, float x, float y)
{
  //The QT drawing coordinate system is reversed from the robot coordinate system in the y direction
  y = -y;

  update_mutex.lock();
  target_locations[rover].push_back(pair<float,float>(x,y));
  update_mutex.unlock();
}


void MapData::addCollectionPoint(string rover, float x, float y)
{
  // The QT drawing coordinate system is reversed from the robot coordinate system in the y direction
  y = -y;

  update_mutex.lock();
  collection_points[rover].push_back(pair<float,float>(x,y));
  update_mutex.unlock();
}

void MapData::setGlobalOffset(bool display)
{
  display_global_offset = display;
}

void MapData::setGlobalOffsetForRover(string rover, float x, float y)
{
  rover_global_offsets[rover] = pair<float,float>(x,y);
}

void MapData::clear()
{
  update_mutex.lock();

  ekf_rover_path.clear();
  encoder_rover_path.clear();
  gps_rover_path.clear();
  global_offset_ekf_rover_path.clear();
  global_offset_encoder_rover_path.clear();
  global_offset_gps_rover_path.clear();
  target_locations.clear();
  collection_points.clear();

  update_mutex.unlock();
}

void MapData::clear(string rover)
{
  update_mutex.lock();

  ekf_rover_path[rover].clear();
  encoder_rover_path[rover].clear();
  gps_rover_path[rover].clear();
  global_offset_ekf_rover_path[rover].clear();
  global_offset_encoder_rover_path[rover].clear();
  global_offset_gps_rover_path[rover].clear();
  target_locations[rover].clear();
  collection_points[rover].clear();

  ekf_rover_path.erase(rover);
  encoder_rover_path.erase(rover);
  gps_rover_path.erase(rover);
  target_locations.erase(rover);
  collection_points.erase(rover);

  update_mutex.unlock();
}

std::vector< std::pair<float,float> >* MapData::getEKFPath(std::string rover_name)
{
  if(display_global_offset)
  {
    return &global_offset_ekf_rover_path[rover_name];
  }

  return &ekf_rover_path[rover_name];
}

std::vector< std::pair<float,float> >* MapData::getGPSPath(std::string rover_name)
{
  if(display_global_offset)
  {
    return &global_offset_gps_rover_path[rover_name];
  }

  return &gps_rover_path[rover_name];
}

std::vector< std::pair<float,float> >* MapData::getEncoderPath(std::string rover_name)
{
  if(display_global_offset)
  {
    return &global_offset_encoder_rover_path[rover_name];
  }

  return &encoder_rover_path[rover_name];
}

std::vector< std::pair<float,float> >* MapData::getTargetLocations(std::string rover_name)
{
  return &target_locations[rover_name];
}

std::vector< std::pair<float,float> >* MapData::getCollectionPoints(std::string rover_name)
{
  return &collection_points[rover_name];
}

// These functions report the maximum and minimum map values seen. This is useful for the GUI when it is calculating the map coordinate system.
float MapData::getMaxGPSX(string rover_name)
{
  if(display_global_offset)
  {
    return max_gps_seen_x[rover_name] + rover_global_offsets[rover_name].first;
  }

  return max_gps_seen_x[rover_name];
}

float MapData::getMaxGPSY(string rover_name)
{
  if(display_global_offset)
  {
    return max_gps_seen_y[rover_name] - rover_global_offsets[rover_name].second;
  }

  return max_gps_seen_y[rover_name];
}

float MapData::getMinGPSX(string rover_name)
{
  if(display_global_offset)
  {
    return min_gps_seen_x[rover_name] + rover_global_offsets[rover_name].first;
  }

  return min_gps_seen_x[rover_name];
}

float MapData::getMinGPSY(string rover_name)
{
  if(display_global_offset)
  {
    return min_gps_seen_y[rover_name] - rover_global_offsets[rover_name].second;
  }

  return min_gps_seen_y[rover_name];
}

float MapData::getMaxEKFX(string rover_name)
{
  if(display_global_offset)
  {
    return max_ekf_seen_x[rover_name] + rover_global_offsets[rover_name].first;
  }

  return max_ekf_seen_x[rover_name];
}

float MapData::getMaxEKFY(string rover_name)
{
  if(display_global_offset)
  {
    return max_ekf_seen_y[rover_name] - rover_global_offsets[rover_name].second;
  }

  return max_ekf_seen_y[rover_name];
}

float MapData::getMinEKFX(string rover_name)
{
  if(display_global_offset)
  {
    return min_ekf_seen_x[rover_name] + rover_global_offsets[rover_name].first;
  }

  return min_ekf_seen_x[rover_name];
}

float MapData::getMinEKFY(string rover_name)
{
  if(display_global_offset)
  {
    return min_ekf_seen_y[rover_name] - rover_global_offsets[rover_name].second;
  }

  return min_ekf_seen_y[rover_name];
}

float MapData::getMaxEncoderX(string rover_name)
{
  if(display_global_offset)
  {
    return max_encoder_seen_x[rover_name] + rover_global_offsets[rover_name].first;
  }

  return max_encoder_seen_x[rover_name];
}

float MapData::getMaxEncoderY(string rover_name)
{
  if(display_global_offset)
  {
    return max_encoder_seen_y[rover_name] - rover_global_offsets[rover_name].second;
  }

  return max_encoder_seen_y[rover_name];
}

float MapData::getMinEncoderX(string rover_name)
{
  if(display_global_offset)
  {
    return min_encoder_seen_x[rover_name] + rover_global_offsets[rover_name].first;
  }

  return min_encoder_seen_x[rover_name];
}

float MapData::getMinEncoderY(string rover_name)
{
  if(display_global_offset)
  {
    return min_encoder_seen_y[rover_name] - rover_global_offsets[rover_name].second;
  }

  return min_encoder_seen_y[rover_name];
}

void MapData::lock()
{
  update_mutex.lock();
}

void MapData::unlock()
{
  update_mutex.unlock();
}


MapData::~MapData()
{
  clear();
}
