#include <Pathfinding/Pather.hpp>
#include "Pathfinding/Objectifier.hpp"
#include <chrono>

#include <iostream>
inline double distance(double dx, double dy) {
    return std::pow(std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5);
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



std::vector<PointLocation::Vertex::Point> string_pull(const std::vector<PointLocation::Vertex::Point> portals) {
	PointLocation::Vertex::Point portalApex;
	PointLocation::Vertex::Point portalLeft;
	PointLocation::Vertex::Point portalRight;
	size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;
	std::vector<PointLocation::Vertex::Point> output = std::vector<PointLocation::Vertex::Point>();
	portalApex = portals[0];
	portalLeft = portals[0];
	portalRight = portals[1];

	output.emplace_back(portalApex);
	for (size_t i = 1; i < portals.size() / 2; ++i) {
		const PointLocation::Vertex::Point left = portals[i * 2];
		const PointLocation::Vertex::Point right = portals[i * 2 + 1];
		if (triarea2(portalApex, portalRight, right) <= 0.0) {
			if (vequal(portalApex, portalRight) || triarea2(portalApex, portalLeft, right) > 0.0) {
				portalRight = right;
				rightIndex = i;
			} else {
				output.emplace_back(portalLeft);
				
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
				output.emplace_back(portalRight);
				
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
	output.emplace_back(portals[(portals.size() / 2 - 1) * 2]);
	return output;
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
	for (int i = 0; i < output->numberoftriangles; i++) {
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
			int first_neigh = -1, second_neigh = -1, third_neigh = -1;
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
	// TriangleManipulator::write_node_file("Maps/" + info->name + ".node", output);
	// TriangleManipulator::write_ele_file("Maps/" + info->name + ".ele", output);
	// TriangleManipulator::write_neigh_file("Maps/" + info->name + ".neigh", output);
	// TriangleManipulator::write_edge_file("Maps/" + info->name + ".edge", output);
	// this->write_to_file(info->name);
}

void MapPather::write_to_file(std::string map_name) {
	this->graph.write_to_binary_file("Maps/" + map_name + ".plgi");
	std::string file_name = "Maps/" + map_name + ".pather";
	TriangleManipulator::binary_writer<> writer = TriangleManipulator::binary_writer<>(file_name.c_str());
	writer.write(this->children.size());
	writer.write(this->neighbhors.size());
	for (size_t i = 0; i < this->children.size(); i++) {
		writer.write<double>(centers[i].x);
		writer.write<double>(centers[i].y);
	}
	writer.write_array(roots.data(), roots.size());
	for (size_t i = 0; i < this->children.size(); i++) {
		std::set<int>& children_of_i = this->children[i];
		writer.write(children_of_i.size());
		for (unsigned int child : children_of_i) {
			writer.write(child);
		}
	}
	for (unsigned int i = 0; i < neighbhors.size(); i++) {
		writer.write(neighbhors[i]);
	}
	writer.close();
}

MapPather::MapPather(std::string map_name) : children(), graph(), roots(), neighbhors(), centers() {
	this->mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:MapPather(" + map_name + ")");
	this->graph = PointLocation::GraphInfo();
	std::string plgi_name = "Maps/" + map_name + ".plgi";
	this->graph.read_from_binary_file(plgi_name);
	std::string file_name = "Maps/" + map_name + ".pather";
	TriangleManipulator::binary_reader<> reader = TriangleManipulator::binary_reader<>(file_name.c_str());
	size_t children_count = reader.read<size_t>();
	size_t neigh_count = reader.read<size_t>();
	centers.reserve(children_count);
	for (size_t i = 0; i < children_count; i++) {
		double x = reader.read<double>();
		double y = reader.read<double>();
		centers.emplace_back(x, y);
	}
	roots.resize(children_count);
	reader.read_array(roots.data(), children_count);
	for (size_t i = 0; i < children_count; i++) {
		std::set<int>& children_of_i = this->children.emplace_back();
		size_t child_count = reader.read<size_t>();
		for (size_t i = 0; i < child_count; i++) {
			children_of_i.emplace(reader.read<unsigned int>());
		}
	}
	for (unsigned int i = 0; i < neigh_count; i++) {
		const auto second = reader.read<std::tuple<int, int, int>>();
		neighbhors.emplace_back(second);
	}
	reader.close();
}

double MapPather::dist_sq(int a, int b) {
	auto A = centers[a], B = centers[b];
	return std::pow(std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2), 0.5);
}
MapPather::PathResult* MapPather::path(PointLocation::Vertex::Point BEGIN, PointLocation::Vertex::Point END) {
	auto t1 = std::chrono::high_resolution_clock::now();
	int start = graph.locate_point(BEGIN),
		end = graph.locate_point(END);
	if (start == -1 || end == -1) {
		return new PathResult{ PathResult::FAIL };
	}
	unsigned int real_start = start; // roots[start];
	unsigned int real_end = end; // roots[end];
	auto comp = [](const Node& first, const Node& second) {
		return (first.distance + first.dist_goal) > (second.distance + second.dist_goal);
	};
	std::priority_queue<Node, std::deque<Node>, decltype(comp)> queue = std::priority_queue<Node, std::deque<Node>, decltype(comp)>();
	queue.emplace(0, real_end, dist_sq(real_end, real_start));
	std::map<unsigned int, unsigned int> PARENTS = std::map<unsigned int, unsigned int>();
	PARENTS.emplace(real_end, -1);
	std::map<unsigned int, double> DISTANCES = std::map<unsigned int, double>();
	bool found = false;
	while (!queue.empty()) {
		const Node curr = queue.top();
		if (curr.identifier == real_start) {
			found = true;
			break;
		}
		int first_neigh, second_neigh, third_neigh;
		std::tie(first_neigh, second_neigh, third_neigh) = neighbhors[curr.identifier];
		if (first_neigh != -1) {
			if (!PARENTS.contains(first_neigh)) {
				const double dist = curr.distance + dist_sq(curr.identifier, first_neigh);
				PARENTS.emplace(first_neigh, curr.identifier);
				DISTANCES.emplace(first_neigh, dist);
				queue.emplace(dist, first_neigh, dist_sq(first_neigh, real_start));
			} else {
				const double dist = curr.distance + dist_sq(curr.identifier, first_neigh);
				if (dist < DISTANCES[first_neigh]) {	
					PARENTS.emplace(first_neigh, curr.identifier);
					DISTANCES.emplace(first_neigh, dist);
				}
			}
		}
		if (second_neigh != -1) {
			if (!PARENTS.contains(second_neigh)) {
				const double dist = curr.distance + dist_sq(curr.identifier, second_neigh);
				PARENTS.emplace(second_neigh, curr.identifier);
				DISTANCES.emplace(second_neigh, dist);
				queue.emplace(dist, second_neigh, dist_sq(second_neigh, real_start));
			} else {
				const double dist = curr.distance + dist_sq(curr.identifier, second_neigh);
				if (dist < DISTANCES[second_neigh]) {	
					PARENTS.emplace(second_neigh, curr.identifier);
					DISTANCES.emplace(second_neigh, dist);
				}
			}
		}
		if (third_neigh != -1) {
			if (!PARENTS.contains(third_neigh)) {
				const double dist = curr.distance + dist_sq(curr.identifier, third_neigh);
				PARENTS.emplace(third_neigh, curr.identifier);
				DISTANCES.emplace(third_neigh, dist);
				queue.emplace(dist, third_neigh, dist_sq(third_neigh, real_start));
			} else {
				const double dist = curr.distance + dist_sq(curr.identifier, third_neigh);
				if (dist < DISTANCES[third_neigh]) {	
					PARENTS.emplace(third_neigh, curr.identifier);
					DISTANCES.emplace(third_neigh, dist);
				}
			}
		}
		queue.pop();
	}
	if (!found) {
		return new PathResult{ PathResult::FAIL };
	}
	PathResult* result = new PathResult{ PathResult::SUCCESS };
	std::vector<PointLocation::Vertex::Point> portals = std::vector<PointLocation::Vertex::Point>();
	portals.emplace_back(BEGIN);
	portals.emplace_back(BEGIN);
	unsigned int current_node = real_start;
	while (PARENTS.at(current_node) != (unsigned int)-1) {
		auto pair = portal(current_node, PARENTS.at(current_node));
		
		portals.emplace_back(pair.second);
		portals.emplace_back(pair.first);
		current_node = PARENTS.at(current_node);
	}
	portals.emplace_back(END);
	portals.emplace_back(END);
	std::vector<PointLocation::Vertex::Point> pulled = string_pull(portals);
	result->path = pulled;
	// result->path.emplace_back(centers[current_node]);
    auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	// result->path.insert(result->path.begin(), BEGIN);
	// result->path.emplace_back(END);
	// std::transform(children[real_end].begin(), std::find(children[real_end].begin(), children[real_end].end(), end), std::back_inserter(result->path), [&](const int location) {
	// 	return this->centers[location];
	// });
	for (PointLocation::Vertex::Point& point : result->path) {
		
		mLogger->info("{" +  std::to_string(point.x) + "," +  std::to_string(point.y) + "}");
	}
	mLogger->info("Pathfinding took: " + std::to_string(ms_double.count()) + "ms");
	return result;
}


Pather::Pather(GameData* data) : map_to_id(), id_to_map(), door_map_in(), door_map_out(), door_spawn_in(), door_spawn_out(), doors_in_map(), door_in(), door_out(), data(data), maps() {
	if (data->was_cached && false) {
		const nlohmann::json& geo = data->at("geometry");
		for (auto& [key, value] : geo.items()) {
			if (value.is_array() && value["x_lines"].size() > 1) {
				this->maps.emplace(key, key);
			}
		}
	} else {
		const nlohmann::json& geo = data->at("geometry");
		for (auto& [key, value] : geo.items()) {
			if (value["x_lines"].is_array()) {
				int id = map_to_id.size();
				map_to_id.emplace(key, id);
				id_to_map.emplace_back(key);
				mLogger->info("Registering map {} with id {}", key, id);
				
				std::shared_ptr<MapProcessing::MapInfo> info = MapProcessing::parse_map(value);
				info->name = key;
				const nlohmann::json& spawns = data->at("maps")[key]["spawns"];
				info->spawns = std::vector<std::pair<double, double>>();
				info->spawns.reserve(spawns.size());
				for (const nlohmann::json& entry : spawns) {
					info->spawns.emplace_back(entry[0].get<double>(), entry[1].get<double>());
				}
				this->maps.emplace(info->name, info);
			} else {
				mLogger->info("Skipping map {}", key);
			}
		}
		// A second pass to generate door paths.
		doors_in_map.resize(maps.size());
		for (auto& [key, value] : geo.items()) {
			if (map_to_id.contains(key)) {
				int map_id = map_to_id.at(key);
				const nlohmann::json& spawns = data->at("maps")[key]["spawns"];
				const nlohmann::json& doors = data->at("maps")[key]["doors"];
				if (doors.is_array()) {
					int index = 0;
					// doors_in_map[map_id] = std::vector<int>();
					doors_in_map[map_id].reserve(doors.size());
					for (auto& door : doors) {
						std::string out_map = door[4].get<std::string>();
						if (map_to_id.contains(out_map)) {
							int id = NEXT_DOOR_ID++;
							doors_in_map[map_id].emplace_back(id);
							int out_map_id = map_to_id.at(out_map);
							const nlohmann::json& in_spawn = spawns.at(door.at(6).get<int>());
							const nlohmann::json& out_spawn = data->at("maps").at(out_map).at("spawns").at(door.at(5).get<int>());
							door_spawn_in.emplace_back(door.at(6).get<int>());
							door_spawn_out.emplace_back(door.at(5).get<int>());
							double in_x = in_spawn.at(0).get<double>(),
								in_y = in_spawn.at(1).get<double>(),
								out_x = out_spawn.at(0).get<double>(),
								out_y = out_spawn.at(1).get<double>();
							door_in.emplace_back(in_x, in_y);
							door_out.emplace_back(out_x, out_y);
							door_map_in.emplace_back(map_id);
							door_map_out.emplace_back(out_map_id);
						} else {
							mLogger->warn("Door {} in map {} leads to invalid map {}", index, key, out_map);
						}
						index++;
					}
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
		mLogger->info(std::get<1>(tup) + " -> " + std::get<3>(tup));
	}
	mLogger->info("Pathfinding took: " + std::to_string(ms_double.count()) + "ms");
	return result;
}