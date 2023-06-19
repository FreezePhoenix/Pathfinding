#pragma once

#include "albot/ServiceInterface.hpp"
#include "TriangleManipulator/PointLocation.hpp"

struct PathfindArguments {
	typedef PointLocation::Vertex::Point Point;
	// MUST. BE. SAME. AS. MAP.PATHER. PATH. RESULT. 
	struct MapPathResult {
		enum Status {
			FAIL = 		0,
			SUCCESS =	1,
		} state;
		std::vector<Point> path;
	};
	typedef std::tuple<Point, std::string, Point, std::string> DoorTuple;
	Point start;
	Point end;
	std::string start_map;
	std::string end_map;
};