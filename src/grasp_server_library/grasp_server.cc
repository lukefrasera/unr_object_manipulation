/*Copyright 2016 Luke Fraser

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
#include "grasp_server_library/grasp_server.h"

namespace grasplib {
Pose::Pose() {}
Pose::~Pose() {}

bool Pose::SetPose() {}
moveit_msgs::Pose Pose::GetPose() {}
Vec4f Pose::GetPoseNormal(Pose pose) {}
Vec4f Pose::GetPoseNormal() {}


Grasp::Grasp() {}
Grasp::~Grasp() {}



ObjectPickPlace::ObjectPickPlace() {}
ObjectPickPlace::~ObjectPickPlace() {}

bool ObjectPickPlace::GenerateGraspFromPose() {}
bool ObjectPickPlace::GetPick() {}
bool ObjectPickPlace::GetPlace() {}
bool ObjectPickPlace::SetPick() {}
bool ObjectPickPlace::SetPlace() {}
bool ObjectPickPlace::SetType() {}
bool ObjectPickPlace::SetName() {}


GraspServer::GraspServer() {}
GraspServer::~GraspServer() {}

bool GraspServer::AddObject(std::string object) {}
bool GraspServer::RemoveObject(std::string object) {}
bool GraspServer::LoadObjects(std::string filename) {}
bool GraspServer::LoadObjects(std::vector<ObjectPickPlace> objects) {}
bool GraspServer::SaveObjects(std::string filename) {}
bool GraspServer::SaveObjects(
    std::string filename, std::vector<ObjectPickPlace> objects) {}
bool GraspServer::MergeObjects(std::string filename) {}
bool GraspServer::MergeObjects(
    std::string filename, std::vector<ObjectPickPlace> objects) {}
Pose GraspServer::GetArmPose(std::string arm) {}
Grasp GraspServer::GenerateGraspFromPose(Pose pose) {}
ObjectPickPlace GraspServer::GetObject(std::string object) {}
std::vector<ObjectPickPlace> GraspServer::GetObjects(
    std::vector<std::string> objects) {}
}  // namespace grasplib
