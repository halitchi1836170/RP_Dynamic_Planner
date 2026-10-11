#pragma once

#include "marrtino_core/io/map_loader.hpp"

namespace marrtino::mapping {

    grid::Grid<float> computeDistanceTransform(const io::OccupancyGrid& map, std::int8_t occupied_threshold = io::kOccupiedThreshold);

} // namespace marrtino::mapping