#pragma once

#ifndef ALBOT_PATHFINDING_PATHER_HPP_
#define ALBOT_PATHFINDING_PATHER_HPP_

#include "Pathfinding/Service.hpp"
#include "albot/MapProcessing/MapProcessing.hpp"
#include "TriangleManipulator/TriangleManipulator.hpp"
#include "TriangleManipulator/ShapeManipulator.hpp"
#include <queue>
#include <stack>


#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

class MapPather {
	struct Node {
		double distance;
		int identifier;
		double dist_goal;
		Node(double a, int b, double c) : distance(a), identifier(b), dist_goal(c) {
		};
	};
		std::vector<std::set<int>> children;
		std::vector<int> roots;
		std::map<int, std::tuple<int, int, int>> neighbhors;
		std::vector<PointLocation::Vertex::Point> centers;
		PointLocation::GraphInfo graph;
		void write_to_file(std::string);
		double dist_sq(int, int);
		std::shared_ptr<spdlog::logger> mLogger;
	public:
		struct PathResult {
			enum Status {
				FAIL = 		0,
				SUCCESS =	1,
			} state;
			std::vector<PointLocation::Vertex::Point> path;
		};
		MapPather(std::shared_ptr<MapProcessing::MapInfo> info);
		MapPather(std::string map_name);
		std::shared_ptr<PathResult> path(PointLocation::Vertex::Point, PointLocation::Vertex::Point);
};

class Pather {
	
		GameData* data;
	public:
		std::shared_ptr<spdlog::logger> mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:Pather");
		std::map<std::string, MapPather> maps;
		Pather(GameData* data);
};

#endif /* ALBOT_PATHFINDING_PATHER_HPP_ */