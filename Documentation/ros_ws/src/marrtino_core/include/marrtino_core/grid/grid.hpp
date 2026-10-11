#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>
#include <algorithm>

#include "marrtino_core/grid/grid_geometry.hpp"

namespace marrtino::grid {

    template<typename T>
    class Grid{

        public:
        Grid() = default;

        explicit Grid(GridGeometry geometry, const T& initial = T{}):geometry_(geometry), cells_(geometry.cellCount(), initial){}

        T& at(Cell c){
            if(!geometry_.contains(c)){
                throw std::out_of_range("Cell is out of grid bounds");
            }
            return cells_[geometry_.cellToIndex(c)];
        }

        const T& at(Cell c) const{
            if(!geometry_.contains(c)){
                throw std::out_of_range("Cell is out of grid bounds");
            }
            return cells_[geometry_.cellToIndex(c)];
        }

        T& operator()(Cell c){
            return cells_[geometry_.cellToIndex(c)];
        }

        const T& operator()(Cell c) const{
            return cells_[geometry_.cellToIndex(c)];
        }

        T& operator[](std::size_t index) { return cells_[index]; }
        const T& operator[](std::size_t index) const { return cells_[index]; }

        [[nodiscard]] const GridGeometry& geometry() const { return geometry_; }

        [[nodiscard]] std::vector<T>& cells() { return cells_; }
        [[nodiscard]] const std::vector<T>& cells() const { return cells_; }

        [[nodiscard]] std::size_t size() const { return cells_.size(); }

        void fill(const T& value) {
            std::fill(cells_.begin(), cells_.end(), value);
        }

        private:
        GridGeometry geometry_{};
        std::vector<T> cells_{};

    };

} // namespace marrtino::grid