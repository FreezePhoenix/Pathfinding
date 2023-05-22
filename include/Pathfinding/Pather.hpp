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
		std::shared_ptr<spdlog::logger> mLogger;
		void write_to_file(std::string);
		double dist_sq(int, int);
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
		PathResult* path(PointLocation::Vertex::Point, PointLocation::Vertex::Point);
};

class Pather {
		struct Node {
			double distance;
			int identifier;
		};
		std::unordered_map<std::string, int> map_to_id;
		std::vector<std::string> id_to_map;
		std::vector<int> door_map_in;
		std::vector<int> door_map_out;
		std::vector<int> door_spawn_in;
		std::vector<int> door_spawn_out;
		std::vector<std::vector<int>> doors_in_map;
		std::vector<PointLocation::Vertex::Point> door_in;
		std::vector<PointLocation::Vertex::Point> door_out;
		int NEXT_DOOR_ID = 0;
		GameData* data;
		inline std::vector<int>& get_doors(int map) {
			return doors_in_map.at(map);
		}
		inline std::vector<int>& get_neighbhors(int door) {
			return get_doors(door_map_out.at(door));
		};
		double dist_sq(int first, int second) {
			auto _first = door_out.at(first);
			auto _second = door_in.at(first);
			return std::pow(std::pow(_first.x - _second.x, 2) + std::pow(_first.y - _second.y, 2), 0.5);
		}
	public:
		std::shared_ptr<spdlog::logger> mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:Pather");
		std::map<std::string, MapPather> maps;
		Pather(GameData* data);
		std::vector<std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>>* path_doors(std::string, std::string);
};

#endif /* ALBOT_PATHFINDING_PATHER_HPP_ */