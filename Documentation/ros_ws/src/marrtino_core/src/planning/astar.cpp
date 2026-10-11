#include "marrtino_core/planning/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "marrtino_core/grid/neighbors.hpp"

namespace marrtino::planning{

    namespace {

        constexpr std::size_t kNoIndex = static_cast<std::size_t>(-1);

        std::size_t findNearestReachableIndex(const grid::Grid<float>& edt, std::size_t index, const AStarParams& params){
            const float resolution = static_cast<float>(edt.geometry().resolution());

            if(edt[index] * resolution >= params.relax_clearance){
                return index;
            }

            grid::Cell origin = edt.geometry().indexToCell(index);
            int maxRing = static_cast<int>(std::ceil(params.relax_radius / resolution));

            for(int ring = 1; ring <= maxRing; ++ring){
                std::size_t best = kNoIndex;
                float bestClearance = -1.0f;

                for(int dx = -ring; dx <= ring; ++dx){
                    for(int dy = -ring; dy <= ring; ++dy){

                        if(std::abs(dx) != ring && std::abs(dy) != ring){
                            continue;
                        }

                        grid::Cell c{origin.x + dx, origin.y + dy};
                        if(!edt.geometry().contains(c)){
                            continue;
                        }

                        std::size_t idx = edt.geometry().cellToIndex(c);
                        float clearance = edt[idx] * resolution;
                        if(clearance >= params.relax_clearance && clearance > bestClearance){
                            bestClearance = clearance;
                            best = idx;
                        }
                    }
                }
                if(best != kNoIndex){
                    return best;
                }
            }
            return kNoIndex;
        }

        float getHeuristic(const grid::GridGeometry& geometry, std::size_t index, std::size_t goalIndex){
            grid::Cell c = geometry.indexToCell(index);
            grid::Cell g = geometry.indexToCell(goalIndex);
            double dx = static_cast<double>(c.x - g.x);
            double dy = static_cast<double>(c.y - g.y);
            return static_cast<float>(std::sqrt(dx*dx + dy*dy));
        }

        bool isWithinEscapeRadius(const grid::GridGeometry& geometry, std::size_t startIndex, std::size_t index, float radius){
            grid::Cell s = geometry.indexToCell(startIndex);
            grid::Cell c = geometry.indexToCell(index);
            float cells = radius / static_cast<float>(geometry.resolution());
            float dx = static_cast<float>(s.x - c.x);
            float dy = static_cast<float>(s.y - c.y);
            return dx*dx + dy*dy <= cells*cells;
        }

        std::vector<grid::Cell> reconstructPath(const grid::GridGeometry& geometry, const std::vector<std::size_t>& parentMap, std::size_t startIndex, std::size_t goalIndex){
            std::vector<grid::Cell> path;
            std::size_t index = goalIndex;

            while(index != kNoIndex){
                path.push_back(geometry.indexToCell(index));
                if(index == startIndex){
                    break;
                }
                index = parentMap[index];
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

    } // namespace

    std::optional<PlanResult> findPath(const grid::Grid<float>& edt, grid::Cell start, grid::Cell goal, const AStarParams& params){

        const grid::GridGeometry& geometry = edt.geometry();

        if(!geometry.contains(start) || !geometry.contains(goal)){
            return std::nullopt;
        }

        std::size_t startIndex = geometry.cellToIndex(start);
        std::size_t goalIndex = geometry.cellToIndex(goal);

        bool goalWasRelaxed = false;
        std::size_t relaxedGoalIndex = findNearestReachableIndex(edt, goalIndex, params);
        if(relaxedGoalIndex == kNoIndex){
            return std::nullopt;
        }
        if(relaxedGoalIndex != goalIndex){
            goalWasRelaxed = true;
            goalIndex = relaxedGoalIndex;
        }

        std::vector<float> costMap(edt.size(), std::numeric_limits<float>::infinity());
        std::vector<std::size_t> parentMap(edt.size(), kNoIndex);

        std::priority_queue<std::pair<float, std::size_t>,
                            std::vector<std::pair<float, std::size_t>>,
                            std::greater<>> Q;

        costMap[startIndex] = 0.0f;
        Q.push({getHeuristic(geometry, startIndex, goalIndex), startIndex});

        while(!Q.empty()){

            auto [priority, index] = Q.top();
            Q.pop();

            if(index == goalIndex){
                return PlanResult{reconstructPath(geometry, parentMap, startIndex, goalIndex),
                                  geometry.indexToCell(goalIndex),
                                  goalWasRelaxed};
            }

            grid::Cell cell = geometry.indexToCell(index);

            for(const grid::Direction& dir : grid::kDirections){

                grid::Cell neighbor{cell.x + dir.dx, cell.y + dir.dy};
                if(!geometry.contains(neighbor)){
                    continue;
                }

                std::size_t neighborIndex = geometry.cellToIndex(neighbor);
                if(edt[neighborIndex] == 0.0f && !isWithinEscapeRadius(geometry, startIndex, neighborIndex, params.start_escape_radius)){
                    continue;
                }

                float transitionCost = dir.cost + params.repulsion_gain / (edt[neighborIndex] + params.epsilon);
                float tentative = costMap[index] + transitionCost;

                if(tentative < costMap[neighborIndex]){
                    costMap[neighborIndex] = tentative;
                    parentMap[neighborIndex] = index;
                    Q.push({tentative + getHeuristic(geometry, neighborIndex, goalIndex), neighborIndex});
                }
            }
        }

        return std::nullopt;
    }

} // namespace marrtino::planning
