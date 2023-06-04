#include "Pathfinding/Objectifier.hpp"

Objectifier::Objectifier(std::shared_ptr<MapProcessing::MapInfo> info): clipper() {
    this->info = info;
    this->holes = std::unordered_set<MapProcessing::Point>();
    this->lines = std::vector<MapProcessing::Line>();
    this->objects = std::vector<std::vector<MapProcessing::Line>>();
};

void Objectifier::init(bool bloat) {
    // Reserve enough memory for al the holes
    holes.reserve((this->info->x_lines.size() + this->info->y_lines.size()) * 2);

    // Insert all of our x_lines into the clipper.
    for (const MapProcessing::AxisLineSegment& line : this->info->x_lines) {
        if (line.range_start != line.range_end) {
            holes.emplace(MapProcessing::Point{line.axis, line.range_start});
            holes.emplace(MapProcessing::Point{line.axis, line.range_end});
            // Here, we insert a path that is a rectangle. It is the line, inflated by the character's base, with an additional unit of distance for spacing.
            clipper.AddPath({ { line.axis + 9, line.range_start - 3 }, { line.axis + 9, line.range_end + 8 }, { line.axis - 9, line.range_end + 8 }, { line.axis - 9, line.range_start - 3 } }, ClipperLib::PolyType::ptSubject, true);
        }
    }

    // Insert all of our y_lines into the clipper.
    for (const MapProcessing::AxisLineSegment& line : this->info->y_lines) {
        if (line.range_start != line.range_end) {
            holes.emplace(MapProcessing::Point{line.range_start, line.axis});
            holes.emplace(MapProcessing::Point{line.range_end, line.axis});
            clipper.AddPath({ { line.range_end + 9, line.axis - 3 }, { line.range_end + 9, line.axis + 8 }, { line.range_start - 9, line.axis + 8 }, { line.range_start - 9, line.axis - 3 } }, ClipperLib::PolyType::ptSubject, true);
        }
    }
}

void Objectifier::run() {
    ClipperLib::Paths polygons;
    clipper.Execute(ClipperLib::ClipType::ctUnion, polygons, ClipperLib::PolyFillType::pftPositive);
    
    size_t num_lines = 0;
    for (const ClipperLib::Path& polygon : polygons) {
        num_lines += polygon.size();
    }
    lines.reserve(num_lines);
    for (const ClipperLib::Path& polygon : polygons) {
        for (size_t i = 0; i < polygon.size() - 1; i++) {
            const ClipperLib::IntPoint& first = polygon[i];
            const ClipperLib::IntPoint& second = polygon[i + 1];
            lines.emplace_back(first.X, first.Y, second.X, second.Y);
        }
        const ClipperLib::IntPoint& first = polygon.back();
        const ClipperLib::IntPoint& second = polygon.front();
        lines.emplace_back(first.X, first.Y, second.X, second.Y);
    }
};
