/*
Copyright 2016 Luke Fraser

This file is part of UNR_Object_Manipulation.

UNR_Object_Manipulation is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

UNR_Object_Manipulation is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with UNR_Object_Manipulation.  If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef INCLUDE_GRASP_SERVER_LIBRARY_GRASP_SERVER_H_
#define INCLUDE_GRASP_SERVER_LIBRARY_GRASP_SERVER_H_
#include <string>
#include <unordered_map>
#include <vector>
#include "moveit_msgs/Grasp.h"
#include "unr_object_manipulation/uomconfig.h"
#include "ros/ros.h"
// #include "uomconfig.h"

namespace grasplib {
class Grasp {
 public:
  Grasp();
  ~Grasp();
 private:
  moveit_msgs::Grasp grasp_;
};
////////////////////////////////////////////////////////////////////////////////
// Class:
//   ObjectPickPlace
// Description:
//   This class stores all grasp and object information needed to produce a full
//   pick and place of the object. It will store the pick grasp as well as the
//   place grasp.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class ObjectPickPlace {
 public:
  ObjectPickPlace();
  ~ObjectPickPlace();

 private:
  std::string name;
  Grasp pick;
  Grasp place;
};

class GraspServer {
 public:
  explicit GraspServer(std::string arm);
  ~GraspServer();

  // Public Facing API functions

  //////////////////////////////////////////////////////////////////////////////
  // Status Functions
  //////////////////////////////////////////////////////////////////////////////
  bool CheckServerState(std::string *status);

  //////////////////////////////////////////////////////////////////////////////
  // Grasp Functions
  //////////////////////////////////////////////////////////////////////////////
  bool AddObject(std::string object);
  bool RemoveObject(std::string object);
  bool LoadGraspFile(std::string filename);
  bool LoadGraspObjects(std::vector<ObjectPickPlace> objects);
  bool SaveGraspFile(std::string filename);
  bool SaveGraspObjects(std::vector<ObjectPickPlace> objects);
  bool MergeGraspFile(std::string filename);
  bool MergeGraspObjects(std::vector<ObjectPickPlace> objects);
  bool GetArmPose();
  bool GenerateGraspFromPose();

  //////////////////////////////////////////////////////////////////////////////
  // Testing Functions
  //////////////////////////////////////////////////////////////////////////////
  bool SendPickPlaceGoal(std::string object);

  //////////////////////////////////////////////////////////////////////////////
  // PARAMETERS
  //////////////////////////////////////////////////////////////////////////////
  bool PostParameters();

 protected:
  ros::NodeHandle nh_;
  std::unordered_map<std::string, ObjectPickPlace> objects_;
};
}  // namespace grasplib
#endif  // INCLUDE_GRASP_SERVER_LIBRARY_GRASP_SERVER_H_
