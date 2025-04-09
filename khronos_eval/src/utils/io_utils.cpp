#include "khronos_eval/utils/io_utils.h"

#include <filesystem>

#include <pcl/io/ply_io.h>

namespace khronos {

bool loadPlyCloud(const std::string& file_path, Points& vertices, std::vector<Color>& colors) {
  if (!std::filesystem::exists(file_path)) {
    LOG(ERROR) << "PLY point cloud file '" << file_path << "' does not exist.";
    return false;
  }

  // Load the ground truth as arbitrary polygon mesh and extract the vertices or points if it is a
  // cloud afterwards.
  pcl::PLYReader ply_reader;
  pcl::PolygonMesh mesh;
  if (ply_reader.read(file_path, mesh) != 0) {
    LOG(ERROR) << "Failed to load PLY point cloud file '" << file_path << "'.";
    return false;
  }

  // Setup the points and neighbor search.
  pcl::PointCloud<pcl::PointXYZRGBA> points;
  pcl::fromPCLPointCloud2(mesh.cloud, points);
  if (points.empty()) {
    LOG(ERROR) << "PLY point cloud '" << file_path << "' has no vertices.";
    return false;
  }

  // Extract the vertices and colors.
  vertices.resize(points.size());
  colors.resize(points.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& point = points[i];
    vertices[i] = point.getVector3fMap();
    colors[i] = Color(point.r, point.g, point.b);
  }
  return true;
}

bool savePlyCloud(const std::string& file_path,
                  const Points& vertices,
                  const std::vector<Color>& colors) {
  return false;
}

}  // namespace khronos
