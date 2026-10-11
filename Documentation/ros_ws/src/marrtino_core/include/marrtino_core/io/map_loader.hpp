#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "marrtino_core/grid/grid.hpp"

namespace marrtino::io {

using OccupancyGrid = grid::Grid<std::int8_t>;


inline constexpr std::int8_t kOccupiedThreshold = 65;


std::optional<OccupancyGrid> loadOccupancyMap(const std::filesystem::path& yaml_path,std::string* error = nullptr);

}  // namespace marrtino::io
