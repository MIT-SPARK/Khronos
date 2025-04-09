#pragma once

#include <string>
#include <vector>

#include <khronos/common/common_types.h>

namespace khronos {

bool loadPlyCloud(const std::string& file_path, Points& vertices, std::vector<Color>& colors);

bool savePlyCloud(const std::string& file_path,
                  const Points& vertices,
                  const std::vector<Color>& colors);

}  // namespace khronos
