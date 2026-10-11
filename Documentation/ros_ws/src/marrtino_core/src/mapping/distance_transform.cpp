#include "marrtino_core/mapping/distance_transform.hpp"

#include <queue>

#include "marrtino_core/grid/neighbors.hpp"

namespace marrtino::mapping{

    grid::Grid<float> computeDistanceTransform(const io::OccupancyGrid& map, std::int8_t occupied_threshold) {

        int width = map.geometry().width();
        int height = map.geometry().height();
        int totCells = width * height;

        grid::Grid<float> distanceGrid(map.geometry(), std::numeric_limits<float>::infinity());

        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> pq;

        for(int i=0; i<totCells; ++i){
            if(map[i]>= occupied_threshold){
                distanceGrid[i] = 0.0f;
                pq.push({0.0f, i});
            }
        }

        while(!pq.empty()){
            auto [dist, index] = pq.top();
            pq.pop();

            if(dist > distanceGrid[index]){
                continue;
            }

            grid::Cell cell = map.geometry().indexToCell(index);
            int cx = cell.x;
            int cy = cell.y;

            for(const grid::Direction& dir : grid::kDirections){
                int nx = cx + dir.dx;
                int ny = cy + dir.dy;

                grid::Cell neighborCell{nx, ny};
                if(!map.geometry().contains(neighborCell)){
                    continue;
                }

                std::size_t neighborIndex = map.geometry().cellToIndex(neighborCell);
                float newDist = dist + dir.cost;
                if(newDist < distanceGrid[neighborIndex]){
                    distanceGrid[neighborIndex] = newDist;
                    pq.push({newDist, static_cast<int>(neighborIndex)});
                }
            }

        }

        return distanceGrid;
    }

} // namespace marrtino::mapping