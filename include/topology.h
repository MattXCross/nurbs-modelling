#pragma once 

#include <vector>
#include <memory>
#include <string>

#include "core.h"

struct CadFace;

struct CadEdge {
  Point3D start, end;
  std::vector<std::weak_ptr<CadFace>> adjacent_faces;
  
  CadEdge(Point3D start_point, Point3D end_point) : start(start_point), end(end_point) {}
};

struct CadFace {
    std::string name;
    std::vector<std::shared_ptr<CadEdge>> boundary_edges;

    explicit CadFace(std::string face_name) : name(std::move(face_name)) {}
};