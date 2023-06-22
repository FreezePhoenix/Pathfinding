#pragma once

#ifndef ALBOT_PATHFINDING_PATHER_HPP_
#define ALBOT_PATHFINDING_PATHER_HPP_

#include "Pathfinding/Service.hpp"
#include "albot/MapProcessing/MapProcessing.hpp"
#include "TriangleManipulator/TriangleManipulator.hpp"
#include "TriangleManipulator/ShapeManipulator.hpp"
#include <queue>
#include <stack>
#include <utility>


#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

class MapPather {
	public:
		constexpr inline static unsigned int NULL_IDENFITIER = (unsigned int)-1;
		std::vector<std::set<unsigned int>> children;
		std::vector<int> roots;
		std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> neighbhors;
		std::vector<PointLocation::Vertex::Point> centers;
		std::shared_ptr<triangulateio> triangle;
		PointLocation::GraphInfo graph;
		std::pair<PointLocation::Vertex::Point, PointLocation::Vertex::Point> portal(unsigned int from, unsigned int to) const {
			int* neigh_ptr = triangle->neighborlist.get();
			double* point_ptr = triangle->pointlist.get();
			unsigned int* tri_ptr = triangle->trianglelist.get();
			
			if (neigh_ptr[from * 3] == to) {
				auto left = tri_ptr[from * 3 + 2];
				auto right = tri_ptr[from * 3 + 1];
				return { {point_ptr[left * 2], point_ptr[left * 2 + 1]}, {point_ptr[right * 2], point_ptr[right * 2 + 1] } };
			}
			if (neigh_ptr[from * 3 + 1] == to) {
				auto left = tri_ptr[from * 3];
				auto right = tri_ptr[from * 3 + 2];
				return { {point_ptr[left * 2], point_ptr[left * 2 + 1]}, {point_ptr[right * 2], point_ptr[right * 2 + 1] } };
			}
			if (neigh_ptr[from * 3 + 2] == to) {
				auto left = tri_ptr[from * 3 + 1];
				auto right = tri_ptr[from * 3];
				return { {point_ptr[left * 2], point_ptr[left * 2 + 1]}, {point_ptr[right * 2], point_ptr[right * 2 + 1] } };
			}
		}
		std::shared_ptr<spdlog::logger> mLogger;
		void write_to_file(std::string map_name);
		void read_from_file(std::string map_name);
		double dist_sq(unsigned int, unsigned int) const;
		double dist_sq(unsigned int, const PointLocation::Vertex::Point&) const;
	public:
		MapPather(std::shared_ptr<MapProcessing::MapInfo> info);
		MapPather(std::string map_name);
		PathfindArguments::MapPathResult path(PointLocation::Vertex::Point, PointLocation::Vertex::Point);
};

class Pather {
	public:
		std::unordered_map<std::string, unsigned int> map_to_id;
		std::vector<std::string> id_to_map;
		GameData* data;
		std::shared_ptr<spdlog::logger> mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:Pather");
		std::vector<MapPather> maps;
		Pather(GameData* data);
};

#endif /* ALBOT_PATHFINDING_PATHER_HPP_ */