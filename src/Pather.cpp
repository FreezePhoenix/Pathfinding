#include <Pathfinding/Pather.hpp>
#include "Pathfinding/Objectifier.hpp"
#include "Pathfinding/PriorityVectorQueue.hpp"
#include "Pathfinding/PriorityFibonacciQueue.hpp"
#include <chrono>

#include <iostream>
inline double distance(double dx, double dy) {
	return std::sqrt(dx * dx + dy * dy);
}

inline double triarea2(PointLocation::Vertex::Point a, PointLocation::Vertex::Point b, PointLocation::Vertex::Point c) {
	const double ax = b.x - a.x;
	const double ay = b.y - a.y;
	const double bx = c.x - a.x;
	const double by = c.y - a.y;
	return bx * ay - ax * by;
}

inline bool vequal(PointLocation::Vertex::Point a, PointLocation::Vertex::Point b) {
	return distance(a.x - b.x, a.y - b.y) < 0.001 * 0.001;
}

inline void string_pull(const std::vector<PointLocation::Vertex::Point>& portals, std::vector<PointLocation::Vertex::Point>& output) {
	PointLocation::Vertex::Point portalApex;
	PointLocation::Vertex::Point portalLeft;
	PointLocation::Vertex::Point portalRight;
	size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;
	portalApex = portals[0];
	portalLeft = portals[0];
	portalRight = portals[1];

	output.push_back(portalApex);
	for (size_t i = 1; i < portals.size() / 2; ++i) {
		const PointLocation::Vertex::Point left = portals[i * 2];
		const PointLocation::Vertex::Point right = portals[i * 2 + 1];
		if (triarea2(portalApex, portalRight, right) <= 0.0) {
			if (vequal(portalApex, portalRight) || triarea2(portalApex, portalLeft, right) > 0.0) {
				portalRight = right;
				rightIndex = i;
			} else {
				if (portalLeft != output.back()) {
					output.push_back(portalLeft);
				}
				
				portalApex = portalLeft;
				apexIndex = leftIndex;
				
				portalLeft = portalApex;
				portalRight = portalApex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;
				i = apexIndex;
				continue;
			}
		}
		if (triarea2(portalApex, portalLeft, left) >= 0.0) {
			if (vequal(portalApex, portalLeft) || triarea2(portalApex, portalRight, left) < 0.0) {
				portalLeft = left;
				leftIndex = i;
			} else {
				if (portalRight != output.back()) {
					output.push_back(portalRight);
				}

				portalApex = portalRight;
				apexIndex = rightIndex;

				portalLeft = portalApex;
				portalRight = portalApex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;
				i = apexIndex;
				continue;
			}
		}
	}
	auto& tail = portals[(portals.size() / 2 - 1) * 2];
	if (tail != output.back()) {
		output.push_back(tail);
	}
}

MapPather::MapPather(std::shared_ptr<MapProcessing::MapInfo> info) : children(), roots(), neighbhors(), centers() {
	this->mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:MapPather(" + info->name + ")");
	Objectifier objectifier(info);
	objectifier.init(true);
	objectifier.run();

	// Really, this method is pretty stupidly complicated, and it changes *a lot*
	// As such, I'm not going to document it. It's intirely internal, and should not be used by a user.
	std::shared_ptr<triangulateio> input = TriangleManipulator::create_instance();
	ShapeManipulator::from_list(objectifier.lines, input);
	if (input->numberofsegments < 3) {
		return;
	}
	
	std::shared_ptr<triangulateio> output = TriangleManipulator::create_instance();
	std::shared_ptr<triangulateio> voutput = TriangleManipulator::create_instance();
	std::shared_ptr<triangulateio> trimmed_input = TriangleManipulator::create_instance();
	
	int num_holes = input->numberofholes = objectifier.holes.size();
	input->holelist = trimalloc<double>(num_holes * 2);
	double* hole_ptr = input->holelist.get();
	for (const auto& hole : objectifier.holes) {
		*hole_ptr++ = hole.x;
		*hole_ptr++ = hole.y;
	}
	triangulate("pznjQ", input, output, nullptr);
	children.resize(output->numberoftriangles);
	roots.resize(output->numberoftriangles);
	centers.reserve(output->numberoftriangles);
	
	std::vector<int> subdomains(output->numberoftriangles);
	std::unordered_map<int, int> mapped_edges = std::unordered_map<int, int>();
	for (unsigned int i = 0; i < output->numberoftriangles; i++) {
		subdomains[i] = 2;
		mapped_edges.insert_or_assign(i, -1);
	}
	int* neigh_ptr = output->neighborlist.get();
	unsigned int* tri_ptr = output->trianglelist.get();
	double* point_ptr = output->pointlist.get();

	std::queue<std::pair<int, int>> queue = std::queue<std::pair<int, int>>();
	for (unsigned int i = 0; i < output->numberoftriangles; i++) {
		int first_tri = neigh_ptr[i * 3];
		int second_tri = neigh_ptr[i * 3 + 1];
		int third_tri = neigh_ptr[i * 3 + 2];
		if (first_tri == -1) {
			subdomains[i] -= 1;
			if (subdomains[i] == 0) {
				const int&& remaining = neigh_ptr[i * 3] + neigh_ptr[i * 3 + 1] + neigh_ptr[i * 3 + 2] - first_tri - mapped_edges.at(i);
				queue.emplace(remaining, i);
			} else {
				mapped_edges.insert_or_assign(i, -1);
			}
		} else {
			children[first_tri].insert(i);
		}
		if (second_tri == -1) {
			subdomains[i] -= 1;
			if (subdomains[i] == 0) {
				const int&& remaining = neigh_ptr[i * 3] + neigh_ptr[i * 3 + 1] + neigh_ptr[i * 3 + 2] - second_tri - mapped_edges.at(i);
				queue.emplace(remaining, i);
			} else {
				mapped_edges.insert_or_assign(i, -1);
			}
		} else {
			children[second_tri].insert(i);
		}
		if (third_tri == -1) {
			subdomains[i] -= 1;
			if (subdomains[i] == 0) {
				const int&& remaining = neigh_ptr[i * 3] + neigh_ptr[i * 3 + 1] + neigh_ptr[i * 3 + 2] - third_tri - mapped_edges.at(i);
				queue.emplace(remaining, i);
			} else {
				mapped_edges.insert_or_assign(i, -1);
			}
		} else {
			children[third_tri].insert(i);
		}
	}

	while (queue.size() > 0) {
		int first, second;
		std::tie(first, second) = queue.front();
		children[first].insert(second);
		children[first].insert(children[second].begin(), children[second].end());
		if (subdomains[first] > 0) {
			if (mapped_edges[first] == -1) {
				subdomains[first] -= 1;
				if (subdomains[first] == 0) {
					int remaining = neigh_ptr[first * 3] + neigh_ptr[first * 3 + 1] + neigh_ptr[first * 3 + 2] - second - mapped_edges.at(first);
					queue.emplace(remaining, first);
				} else {
					mapped_edges.insert_or_assign(first, second);
				}
			}
		}
		queue.pop();
	}
	neighbhors.reserve(output->numberoftriangles);
	for (unsigned int i = 0; i < output->numberoftriangles; i++) {
		const unsigned int first = tri_ptr[i * 3],
			second = tri_ptr[i * 3 + 1],
			third = tri_ptr[i * 3 + 2];

		
		double first_distance = distance(point_ptr[second * 2] - point_ptr[third * 2], point_ptr[second * 2 + 1] - point_ptr[third * 2 + 1]);
		double second_distance = distance(point_ptr[first * 2] - point_ptr[third * 2], point_ptr[first * 2 + 1] - point_ptr[third * 2 + 1]);
		double third_distance = distance(point_ptr[first * 2] - point_ptr[second * 2], point_ptr[first * 2 + 1] - point_ptr[second * 2 + 1]);
		
		const double new_x = (first_distance * point_ptr[first * 2] + second_distance * point_ptr[second * 2] + third_distance * point_ptr[third * 2]) / (first_distance + second_distance + third_distance);
		const double new_y = (first_distance * point_ptr[first * 2 + 1] + second_distance * point_ptr[second * 2 + 1] + third_distance * point_ptr[third * 2 + 1]) / (first_distance + second_distance + third_distance);
		centers.emplace_back(new_x, new_y);
		int subdomain = subdomains[i];
		// TODO: Optimize child nodes.
		if (subdomain != 0 || true) {
			roots[i] = i;
			unsigned int first_neigh = -1, second_neigh = -1, third_neigh = -1;
			// if (neigh_ptr[i * 3] != -1 && subdomains[neigh_ptr[i * 3]] != 0) {
			first_neigh = neigh_ptr[i * 3];
			// }
			// if (neigh_ptr[i * 3 + 1] != -1 && subdomains[neigh_ptr[i * 3 + 1]] != 0) {
			second_neigh = neigh_ptr[i * 3 + 1];
			// }
			// if (neigh_ptr[i * 3 + 2] != -1 && subdomains[neigh_ptr[i * 3 + 2]] != 0) {
			third_neigh = neigh_ptr[i * 3 + 2];
			// }
			neighbhors.emplace_back(first_neigh, second_neigh, third_neigh);
		}
		// if (subdomain == 1) {
		// 	for (unsigned int child : children[i]) {
		// 		if (subdomains[child] == 0) {
		// 			roots[child] = i;
		// 		}
		// 	}
		// }
	}
	this->triangle = output;
	this->graph = output;
	this->graph.process();
	this->graph.map_triangles(output);
	// TriangleManipulator::write_part_file("Maps/" + info->name + ".part", output);
	// TriangleManipulator::write_poly_file("Maps/" + info->name + ".poly", input);
	// TriangleManipulator::write_edge_file("Maps/" + info->name + ".v.edge", temp2);
	// TriangleManipulator::write_node_file("Maps/" + info->name + ".v.node", voutput);
	// TriangleManipulator::write_neigh_file("Maps/" + info->name + ".neigh", output);
	// TriangleManipulator::write_edge_file("Maps/" + info->name + ".edge", output);
	// std::cout << &test << std::endl;
	this->write_to_file(info->name);
}

void MapPather::write_to_file(std::string map_name) {
	this->graph.write_to_binary_file("Maps/" + map_name + ".plgi");
	std::string file_name = "Maps/" + map_name + ".pather";
	TriangleManipulator::binary_writer<true> writer = TriangleManipulator::binary_writer<true>(file_name.c_str());

	writer.write(children.size());
	for (auto& child_entry : children) {
		writer.write(child_entry.size());
		for (auto& child : child_entry) {
			writer.write(child);
		}
	}
	writer.write(roots.size());
	for (auto& root : roots) {
		writer.write(root);
	}
	writer.write(neighbhors.size());
	for (auto& neighbhor_entry : neighbhors) {
		writer.write(neighbhor_entry);
	}
	writer.write(centers.size());
	for (auto& center : centers) {
		writer.write(center);
	}
	writer.close();
	TriangleManipulator::write_neigh_file_binary("Maps/" + map_name + ".neigh.bin", triangle);
	TriangleManipulator::write_node_file_binary("Maps/" + map_name + ".node.bin", triangle);
	TriangleManipulator::write_ele_file_binary("Maps/" + map_name + ".ele.bin", triangle);
	// TODO: Serialize `triangle
}

void MapPather::read_from_file(std::string map_name) {
	this->graph.read_from_binary_file("Maps/" + map_name + ".plgi");
	std::string file_name = "Maps/" + map_name + ".pather";
	TriangleManipulator::binary_reader<true> reader = TriangleManipulator::binary_reader<true>(file_name.c_str());
	size_t children_size = reader.read<size_t>();
	children.reserve(children_size);
	
	for (size_t i = 0; i < children_size; i++) {
		size_t child_entry_size = reader.read<size_t>();
		std::set<int>& child_entry = children.emplace_back();
		for (size_t j = 0; j < child_entry_size; j++) {
			child_entry.emplace(reader.read<int>());
		}
	}
	size_t roots_size = reader.read<size_t>();
	roots.resize(roots_size);
	reader.read_array(roots.data(), roots_size);
	
	size_t neighbhors_size = reader.read<size_t>();
	neighbhors.resize(neighbhors_size);
	reader.read_array(neighbhors.data(), neighbhors_size);

	size_t centers_size = reader.read<size_t>();
	centers.resize(centers_size);
	reader.read_array(centers.data(), centers_size);
	
	reader.close();

	triangle = TriangleManipulator::create_instance();
	TriangleManipulator::read_neigh_file_binary("Maps/" + map_name + ".neigh.bin", triangle);
	TriangleManipulator::read_node_file_binary("Maps/" + map_name + ".node.bin", triangle);
	TriangleManipulator::read_ele_file_binary("Maps/" + map_name + ".ele.bin", triangle);
}

MapPather::MapPather(std::string map_name) : children(), roots(), neighbhors(), centers(), triangle(), graph() {
	this->mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:MapPather(" + map_name + ")");
	this->read_from_file(map_name);
}

inline double MapPather::dist_sq(unsigned int a, unsigned int b) {
	auto& A = centers[a], B = centers[b];
	return distance(B.x - A.x, B.y - A.y);
}
MapPather::PathResult* MapPather::path(PointLocation::Vertex::Point BEGIN, PointLocation::Vertex::Point END) {
	auto t1 = std::chrono::high_resolution_clock::now();
	auto start = graph.locate_point(BEGIN);
	if (!start.has_value()) {
		return new PathResult{ PathResult::FAIL };
	}
	auto end = graph.locate_point(END);
	if (!end.has_value()) {
		return new PathResult{ PathResult::FAIL };
	}
	unsigned int real_start = start.value(); // roots[start];
	unsigned int real_end = end.value(); // roots[end];
	auto ident = [](const Node& first, double traveled, unsigned int identifier) {
		return first.identifier == identifier;
	};
	struct VisitedEntry { // Slightly more compact than an optional
		double distance;
		unsigned int parent;
		bool visited;
		inline void visit(double distance, unsigned int parent) {
			this->distance = distance;
			this->parent = parent;
			this->visited = true;
		}
		inline void visit() {
			this->visited = true;
		}
	};
	sizeof(VisitedEntry);
	auto queue = PriorityVectorQueue<double, Node, std::greater<double>>();
	queue.emplace(dist_sq(real_end, real_start), 0.0, real_end);
	std::vector<VisitedEntry> visited = std::vector<VisitedEntry>(this->neighbhors.size());
	visited[real_end].visit(0.0, NULL_IDENFITIER);
	bool found = false;
	while (!queue.empty()) {
		const Node curr = queue.remove();
		// We do some operations which may involve rewriting the queue, so we pop early.
		if (curr.identifier == real_start) {
			found = true;
			break;
		}
		auto [first_neigh, second_neigh, third_neigh] = neighbhors[curr.identifier];
		if (first_neigh != NULL_IDENFITIER) {
			const double dist = curr.traveled + dist_sq(curr.identifier, first_neigh);
			auto& entry = visited[first_neigh];
			if (!entry.visited) {
				queue.emplace(dist + dist_sq(first_neigh, real_start), dist, first_neigh);
				entry.visit(dist, curr.identifier);
			} else if (dist < entry.distance) {
				queue.raise_priority<decltype(ident)>(dist + dist_sq(first_neigh, real_start), dist, first_neigh);
				entry.visit(dist, curr.identifier);
			}
		}
		if (second_neigh != NULL_IDENFITIER) {
			const double dist = curr.traveled + dist_sq(curr.identifier, second_neigh);
			auto& entry = visited[second_neigh];
			if (!entry.visited) {
				queue.emplace(dist + dist_sq(second_neigh, real_start), dist, second_neigh);
				entry.visit(dist, curr.identifier);
			} else if (dist < entry.distance) {
				// mLogger->info("Uhhh wtf {}, {}", dist, DISTANCES[second_neigh]);
				queue.raise_priority<decltype(ident)>(dist + dist_sq(second_neigh, real_start), dist, second_neigh);
				entry.visit(dist, curr.identifier);				
			}
		}
		if (third_neigh != NULL_IDENFITIER) {
			const double dist = curr.traveled + dist_sq(curr.identifier, third_neigh);
			auto& entry = visited[third_neigh];
			if (!entry.visited) {
				queue.emplace(dist + dist_sq(third_neigh, real_start), dist, third_neigh);
				entry.visit(dist, curr.identifier);
			} else if (dist < entry.distance) {
				queue.raise_priority<decltype(ident)>(dist + dist_sq(third_neigh, real_start), dist, third_neigh);
				entry.visit(dist, curr.identifier);
			}
		}
	}
	if (!found) {
		return new PathResult{ PathResult::FAIL };
	}
	PathResult* result = new PathResult{ PathResult::SUCCESS };
	std::vector<PointLocation::Vertex::Point> portals = std::vector<PointLocation::Vertex::Point>();
	portals.push_back(BEGIN);
	portals.push_back(BEGIN);
	for (unsigned int current_node = real_start, next_node = visited[current_node].parent; next_node != NULL_IDENFITIER; current_node = std::exchange(next_node, visited[next_node].parent)) {
		const auto& pair = portal(current_node, next_node);
		portals.push_back(pair.second);
		portals.push_back(pair.first);
	}
	portals.push_back(END);
	portals.push_back(END);
	string_pull(portals, result->path);
	std::vector<PointLocation::Vertex::Point>& pulled = result->path;
	// pulled.erase(std::unique(pulled.begin(), pulled.end(), [](const PointLocation::Vertex::Point& lhs, const PointLocation::Vertex::Point& rhs) {
	// 	return lhs.x == rhs.x && lhs.y == rhs.y;
	// }), pulled.end());
	// result->path.emplace_back(centers[current_node]);
    auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	for (PointLocation::Vertex::Point& point : result->path) {
		mLogger->info("{{{},{}}}", point.x, point.y);
	}
	mLogger->info("Pathfinding took: {}ms", ms_double.count());
	return result;
}


Pather::Pather(GameData* data) : map_to_id(), id_to_map(), door_map_in(), door_map_out(), door_spawn_in(), door_spawn_out(), doors_in_map(), door_in(), door_out(), data(data), maps() {
	const nlohmann::json& geo = data->operator[]("geometry");
	const nlohmann::json& maps = data->operator[]("maps");
	for (auto& [key, value] : geo.items()) {
		if (value["x_lines"].is_array() && value["x_lines"].size() > 0) {
			unsigned int id = map_to_id.size();
			map_to_id.emplace(key, id);
			id_to_map.emplace_back(key);
			
			std::shared_ptr<MapProcessing::MapInfo> info = MapProcessing::parse_map(value);
			info->name = key;
			const nlohmann::json& json_spawns = maps[key]["spawns"];
			auto& info_spawns = info->spawns;
			info_spawns.reserve(json_spawns.size());
			auto& map_spawns_entry = map_spawns.emplace_back();
			map_spawns_entry.reserve(json_spawns.size());
			for (const nlohmann::json& entry : json_spawns) {
				double x = entry[0].get<double>();
				double y = entry[1].get<double>();
				info_spawns.emplace_back(x, y);
				map_spawns_entry.emplace_back(x, y);
			}
			if (data->was_cached) {
				mLogger->info("Registering cached map {} with id {}", key, id);
				this->maps.emplace(key, key);
			} else {
				mLogger->info("Registering map {} with id {}", key, id);
				this->maps.emplace(key, info);
			}
		} else {
			if (data->was_cached) {
				mLogger->info("Skipping cached map {}", key);
			} else {
				mLogger->info("Skipping map {}", key);
			}
		}
	}
	// A second pass to generate door paths.
	doors_in_map.resize(maps.size());
	for (auto& [key, value] : geo.items()) {
		const nlohmann::json& maps_entry = maps[key];
		if (map_to_id.contains(key)) {
			unsigned int map_id = map_to_id[key];
			std::vector<PointLocation::Vertex::Point>& spawns = map_spawns[map_id];
			// const nlohmann::json& spawns = maps_entry["spawns"];
			const nlohmann::json& doors = maps_entry["doors"];
			if (doors.is_array()) {
				unsigned int index = 0;
				// doors_in_map[map_id] = std::vector<int>();
				doors_in_map[map_id].reserve(doors.size());
				for (auto& door : doors) {
					const std::string& out_map = door[4].get<std::string>();
					if (map_to_id.contains(out_map)) {
						unsigned int id = NEXT_DOOR_ID++;
						doors_in_map[map_id].push_back(id);
						unsigned int out_map_id = map_to_id[out_map];
						unsigned int in_spawn_id = door[6].get<unsigned int>();
						unsigned int out_spawn_id = door[5].get<unsigned int>();
						const auto& in_spawn = spawns[in_spawn_id];
						const auto& out_spawn = map_spawns[out_map_id][out_spawn_id];
						door_spawn_in.push_back(in_spawn_id);
						door_spawn_out.push_back(out_spawn_id);
						door_in.emplace_back(in_spawn);
						door_out.emplace_back(out_spawn);
						door_map_in.push_back(map_id);
						door_map_out.push_back(out_map_id);
					} else {
						mLogger->warn("Door {} in map {} leads to invalid map {}", index, key, out_map);
					}
					index++;
				}
			}
		}
	}
};

std::vector<std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>>* Pather::path_doors(std::string BEGIN, std::string END) {
	auto t1 = std::chrono::high_resolution_clock::now();
	int start = map_to_id.at(BEGIN),
		end = map_to_id.at(END);

	auto comp = [](const Node& first, const Node& second) {
		return (first.distance) > (second.distance);
	};
	std::priority_queue<Node, std::deque<Node>, decltype(comp)> queue = std::priority_queue<Node, std::deque<Node>, decltype(comp)>();
	std::map<int, int> PARENTS = std::map<int,int>();
	std::map<int, double> DISTANCES = std::map<int, double>();
	for (int door : get_doors(end)) {
		// mLogger->info("Initial door: " + std::to_string(door_in[door].x) + "," + std::to_string(door_in[door].y) + ":" + std::to_string(door_map_out[door]));
		PARENTS.emplace(door, -1);
		DISTANCES.emplace(door, 0);
		queue.emplace(0, door);
	}
	bool found = false;
	int current_node = -1;
	while (!queue.empty()) {
		const Node curr = queue.top();
		// mLogger->info("Checking: " + std::to_string(curr.identifier) + " in " + std::to_string(door_map_in[curr.identifier]));
		if (door_map_out[curr.identifier] == start) {
			found = true;
			current_node = curr.identifier;
			break;
		}
		for (int door : get_neighbhors(curr.identifier)) {
			const double dist = curr.distance + dist_sq(curr.identifier, door);
			if (!PARENTS.contains(door)) {
				PARENTS.emplace(door, curr.identifier);
				DISTANCES.emplace(door, dist);
				queue.emplace(dist, door);
			} else {
				if (dist < DISTANCES.at(door)) {	
					PARENTS.emplace(door, curr.identifier);
					DISTANCES.emplace(door, dist);
				}
			}
		}
		queue.pop();
	}
	
	if (!found) {
		return new std::vector<std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>>();
	}
	std::vector<std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>>* result = new std::vector<std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>>();
	
	while (PARENTS.at(current_node) != (unsigned int) -1) {
		result->emplace_back(door_in[current_node], id_to_map[door_map_out[current_node]], door_out[current_node], id_to_map[door_map_in[current_node]]);
		current_node = PARENTS.at(current_node);
	}
	result->emplace_back(door_in[current_node], id_to_map[door_map_out[current_node]], door_out[current_node], id_to_map[door_map_in[current_node]]);
	auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	for (std::tuple<PointLocation::Vertex::Point, std::string, PointLocation::Vertex::Point, std::string>& tup : *result) {
		mLogger->info("{} -> {}", std::get<1>(tup), std::get<3>(tup));
	}
	mLogger->info("Pathfinding took: {}ms", ms_double.count());
	return result;
}