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
	public:
		constexpr inline static unsigned int NULL_IDENFITIER = (unsigned int)-1;
		struct Node {
			double traveled;
			unsigned int identifier;
			inline bool operator==(const Node& other) const {
				return identifier == other.identifier;
			}
		};
		std::vector<std::set<int>> children;
		std::vector<int> roots;
		std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> neighbhors;
		std::vector<PointLocation::Vertex::Point> centers;
		std::shared_ptr<triangulateio> triangle;
		PointLocation::GraphInfo graph;
		std::pair<PointLocation::Vertex::Point, PointLocation::Vertex::Point> portal(int from, int to) {
			int* neigh_ptr = triangle->neighborlist.get();
			double* point_ptr = triangle->pointlist.get();
			unsigned int* tri_ptr = triangle->trianglelist.get();
			if (neigh_ptr[from * 3] == to) {
				auto left = tri_ptr[from * 3 + 1];
				auto right = tri_ptr[from * 3 + 2];
				return { {point_ptr[left * 2], point_ptr[left * 2 + 1]}, {point_ptr[right * 2], point_ptr[right * 2 + 1] } };
			}
			if (neigh_ptr[from * 3 + 1] == to) {
				auto left = tri_ptr[from * 3 + 2];
				auto right = tri_ptr[from * 3];
				return { {point_ptr[left * 2], point_ptr[left * 2 + 1]}, {point_ptr[right * 2], point_ptr[right * 2 + 1] } };
			}
			if (neigh_ptr[from * 3 + 2] == to) {
				auto left = tri_ptr[from * 3];
				auto right = tri_ptr[from * 3 + 1];
				return { {point_ptr[left * 2], point_ptr[left * 2 + 1]}, {point_ptr[right * 2], point_ptr[right * 2 + 1] } };
			}
		}
		std::shared_ptr<spdlog::logger> mLogger;
		void write_to_file(std::string map_name);
		void read_from_file(std::string map_name);
		double dist_sq(unsigned int, unsigned int);
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
			unsigned int identifier;
		};
		std::unordered_map<std::string, unsigned int> map_to_id;
		std::vector<std::vector<PointLocation::Vertex::Point>> map_spawns;
		std::vector<std::string> id_to_map;
		std::vector<unsigned int> door_map_in;
		std::vector<unsigned int> door_map_out;
		std::vector<unsigned int> door_spawn_in;
		std::vector<unsigned int> door_spawn_out;
		std::vector<std::vector<unsigned int>> doors_in_map;
		std::vector<PointLocation::Vertex::Point> door_in;
		std::vector<PointLocation::Vertex::Point> door_out;
		unsigned int NEXT_DOOR_ID = 0;
		GameData* data;
		inline std::vector<unsigned int>& get_doors(unsigned int map) {
			return doors_in_map[map];
		}
		inline std::vector<unsigned int>& get_neighbhors(unsigned int door) {
			return get_doors(door_map_out[door]);
		};
		double dist_sq(unsigned int first, unsigned int second) {
			auto& _first = door_out[first];
			auto& _second = door_in[first];
			return std::pow(std::pow(_first.x - _second.x, 2) + std::pow(_first.y - _second.y, 2), 0.5);
		}
	public:
		std::shared_ptr<spdlog::logger> mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:Pather");
		std::map<std::string, MapPather> maps;
		Pather(GameData* data);
		std::vector<std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>>* path_doors(std::string, std::string);
};

template<>
struct std::hash<MapPather::Node> {
	inline std::size_t operator()(MapPather::Node const& n) const noexcept {
		return n.identifier;
	}
};

#endif /* ALBOT_PATHFINDING_PATHER_HPP_ */