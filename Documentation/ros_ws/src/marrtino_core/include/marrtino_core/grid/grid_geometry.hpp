#pragma once

#include <cstddef>

namespace marrtino::grid {

    struct Cell{
        int x{};
        int y{};
    };

    struct Point2 {
        double x{};
        double y{};
    };

    class GridGeometry{
        public: 

        GridGeometry() = default;

        GridGeometry(int width, int height, double resolution, Point2 origin);

        [[nodiscard]] Cell worldToCell(Point2 p) const;
        [[nodiscard]] Point2 cellToWorld(Cell c) const;

        [[nodiscard]] std::size_t cellToIndex(Cell c) const;
        [[nodiscard]] Cell indexToCell(std::size_t index) const;

        [[nodiscard]] std::size_t worldToIndex(Point2 p) const;

        [[nodiscard]] bool contains(Cell c) const;

        [[nodiscard]] int width() const { return width_; }
        [[nodiscard]] int height() const { return height_; }
        [[nodiscard]] double resolution() const { return resolution_; }
        [[nodiscard]] Point2 origin() const { return origin_; }

        [[nodiscard]] std::size_t cellCount() const;

        private:
        int width_{};
        int height_{};
        double resolution_{1.0};
        Point2 origin_{};
    };

} // namespace marrtino::grid