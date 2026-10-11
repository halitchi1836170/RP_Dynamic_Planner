#include "marrtino_core/grid/grid_geometry.hpp"

#include <cmath>

namespace marrtino::grid {

    GridGeometry::GridGeometry(int width, int height, double resolution, Point2 origin)
    : width_(width), height_(height), resolution_(resolution), origin_(origin) {}

    Cell GridGeometry::worldToCell(Point2 p) const {
        int cx = static_cast<int>(floor((p.x-origin_.x)/resolution_));
        int cy = static_cast<int>(floor((p.y-origin_.y)/resolution_));
        return Cell{cx, cy};
    }

    Point2 GridGeometry::cellToWorld(Cell c) const {
        return Point2{origin_.x+(c.x+0.5)*resolution_,origin_.y+(c.y+0.5)*resolution_};
    }

    std::size_t GridGeometry::cellToIndex(Cell c) const {
        return c.y*(width_)+ c.x;
    }

    Cell GridGeometry::indexToCell(std::size_t index) const {
        int cx = static_cast<int>(index % width_);
        int cy = static_cast<int>(index / width_);
        return Cell{cx, cy};
    }

    std::size_t GridGeometry::worldToIndex(Point2 p) const {
        Cell c = worldToCell(p);
        return cellToIndex(c);
    }

    bool GridGeometry::contains(Cell c) const {
        return c.x >= 0 && c.x < width_ && c.y >= 0 && c.y < height_;
    }

    std::size_t GridGeometry::cellCount() const {
        return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
    }

} // namespace marrtino::grid