#pragma once

#include <cstddef>
#include <vector>

namespace marrtino::obstacles {

    // Un rilevamento grezzo di una singola scansione, in frame MAPPA.
    struct Detection{
        double cx{};
        double cy{};
        double r{};
        int points{};
    };

    // Un ostacolo seguito nel tempo, in frame MAPPA.
    struct Track{
        int id{};
        double cx{};
        double cy{};
        double r{};
        double vx{};
        double vy{};
        int hits{};
        bool confirmed{};
    };

    using Detections = std::vector<Detection>;
    using Tracks = std::vector<Track>;

} // namespace marrtino::obstacles
