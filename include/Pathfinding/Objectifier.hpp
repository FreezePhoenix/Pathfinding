#pragma once

#ifndef ALBOT_OBJECTIFIER_HPP_
#define ALBOT_OBJECTIFIER_HPP_

#include "albot/MapProcessing/MapProcessing.hpp"
#include <unordered_set>
#define use_int32
#include "clipper/clipper.hpp"

class Objectifier {
    public:
        struct RawPoint {
            short x;
            short y;
        };
        ClipperLib::Clipper clipper;
        std::vector<std::vector<MapProcessing::Line>> objects;
        std::vector<MapProcessing::Line> lines;
        std::unordered_set<MapProcessing::Point> holes;
        std::shared_ptr<MapProcessing::MapInfo> info;
        Objectifier(std::shared_ptr<MapProcessing::MapInfo> info);

        /**
         * @brief Run the objectifier on the given data.
         * 
         */
        void init(bool bloat);
        void run();
};

#endif /* ALBOT_OBJECTIFIER_HPP_ */