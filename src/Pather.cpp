#include "Pathfinding/Pather.hpp"
#include "Pathfinding/Objectifier.hpp"
#include "Pathfinding/PriorityVectorQueue.hpp"
#include "TriangleManipulator/TriangleManipulator.hpp"

inline double distance(double dx, double dy) {
	return std::sqrt(dx * dx + dy * dy);
}

inline double triarea2(const PointLocation::Vertex::Point& a, const PointLocation::Vertex::Point& b, const PointLocation::Vertex::Point& c) {
	const double ax = b.x - a.x;
	const double ay = b.y - a.y;
	const double bx = c.x - a.x;
	const double by = c.y - a.y;
	return bx * ay - ax * by;
}

inline void string_pull(const PointLocation::Vertex::Point& start, const PointLocation::Vertex::Point& end, const std::vector<std::pair<PointLocation::Vertex::Point, PointLocation::Vertex::Point>>& portals, std::vector<PointLocation::Vertex::Point>& output) {
	PointLocation::Vertex::Point portalApex = start,
		portalLeft = start,
		portalRight = start;
	size_t apexIndex = 0,
		leftIndex = 0,
		rightIndex = 0;

	output.push_back(portalApex);
	for (size_t i = 0; i < portals.size(); i++) {
		const auto& [left, right] = portals[i];
		if (triarea2(portalApex, portalRight, right) <= 0.0) {
			if (portalApex == portalRight || triarea2(portalApex, portalLeft, right) > 0.0) {
				portalRight = right;
				rightIndex = i;
			} else {
				if (portalLeft != portalApex) {
					output.push_back(portalLeft);
					portalApex = portalLeft;
				}
				
				portalRight = portalApex;
				apexIndex = leftIndex;
				rightIndex = apexIndex;
				i = apexIndex;
				continue;
			}
		}
		if (triarea2(portalApex, portalLeft, left) >= 0.0) {
			if (portalApex == portalLeft || triarea2(portalApex, portalRight, left) < 0.0) {
				portalLeft = left;
				leftIndex = i;
			} else {
				if (portalRight != portalApex) {
					output.push_back(portalRight);
					portalApex = portalRight;
				}
				portalLeft = portalApex;
				apexIndex = rightIndex;
				leftIndex = apexIndex;
				i = apexIndex;
				continue;
			}
		}
	}
	if (end != portalApex) {
		output.push_back(end);
	}
}

MapPather::MapPather(std::shared_ptr<MapProcessing::MapInfo> info) : children(), roots(), neighbhors(), centers() {
	this->mLogger = spdlog::stdout_color_mt("Pathfinding:MapPather(" + info->name + ")");
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
	input->holelist = trimallocarr<double>(num_holes * 2);
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
	
	this->write_to_file(info->name);
}

void MapPather::write_to_file(std::string map_name) {
	this->graph.write_to_binary_file("Maps/" + map_name + ".plgi");
	std::string file_name = "Maps/" + map_name + ".pather";
	TriangleManipulator::binary_writer writer(file_name.c_str());

	writer.write(roots.size());
	writer.write_array(roots.data(), roots.size());
	
	writer.write(neighbhors.size());
	writer.write_array(neighbhors.data(), neighbhors.size());
	
	writer.write(centers.size());
	writer.write_array(centers.data(), centers.size());
	
	writer.close();
	
	TriangleManipulator::write_neigh_file_binary("Maps/" + map_name + ".neigh.bin", triangle);
	TriangleManipulator::write_node_file_binary("Maps/" + map_name + ".node.bin", triangle);
	TriangleManipulator::write_ele_file_binary("Maps/" + map_name + ".ele.bin", triangle);
	// TODO: Serialize `triangle
}

void MapPather::read_from_file(std::string map_name) {
	this->graph.read_from_binary_file("Maps/" + map_name + ".plgi");
	std::string file_name = "Maps/" + map_name + ".pather";
	TriangleManipulator::binary_reader reader(file_name.c_str());
	
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
	this->mLogger = spdlog::stdout_color_mt("Pathfinding:MapPather(" + map_name + ")");
	this->read_from_file(map_name);
}

inline double MapPather::dist_sq(unsigned int a, unsigned int b) const {
	const auto& A = centers[a], B = centers[b];
	return distance(B.x - A.x, B.y - A.y);
}
inline double MapPather::dist_sq(unsigned int a, const PointLocation::Vertex::Point& B) const {
	const auto& A = centers[a];
	return distance(B.x - A.x, B.y - A.y);
}
PathfindArguments::MapPathResult MapPather::path(PointLocation::Vertex::Point BEGIN, PointLocation::Vertex::Point END) {
	// auto t1 = std::chrono::high_resolution_clock::now();
	auto start = graph.locate_point(BEGIN);
	if (!start.has_value()) {
		return { PathfindArguments::MapPathResult::FAIL };
	}
	auto end = graph.locate_point(END);
	if (!end.has_value()) {
		return { PathfindArguments::MapPathResult::FAIL };
	}
	unsigned int real_start = start.value(); // roots[start];
	unsigned int real_end = end.value(); // roots[end];
	PointLocation::Vertex::Point REAL_START = centers[real_start];
	struct VisitedEntry { // Slightly more compact than an optional
		double distance = 0.0;
		unsigned int parent = 0;
		enum STATUS {
			EMPTY,
			QUEUED,
			VISITED
		} status = EMPTY;
		constexpr void queue(double distance, unsigned int parent) {
			this->distance = distance;
			this->parent = parent;
			this->status = QUEUED;
		}
		constexpr void requeue(double distance, unsigned int parent) {
			this->distance = distance;
			this->parent = parent;
		}
		constexpr void visit() {
			this->status = VISITED;
		}
	};
	PriorityVectorQueue<double, unsigned int, std::greater<double>> queue;
	queue.emplace(dist_sq(real_end, real_start), real_end);
	std::vector<VisitedEntry> visited(this->neighbhors.size());
	visited[real_end].queue(0.0, NULL_IDENFITIER);
	bool found = false;
	while (!queue.empty()) {
		const unsigned int curr_identifier = queue.top();
		// We do some operations which may involve rewriting the queue, so we pop early.
		if (curr_identifier == real_start) {
			found = true;
			break;
		}
		queue.pop();
		auto& curr_entry = visited[curr_identifier];
		curr_entry.visit();
		auto [first_neigh, second_neigh, third_neigh] = neighbhors[curr_identifier];
		if (first_neigh != NULL_IDENFITIER && curr_entry.parent != first_neigh) {
			auto& entry = visited[first_neigh];
			if (entry.status != VisitedEntry::VISITED) {
				const double dist = curr_entry.distance + dist_sq(curr_identifier, first_neigh);
				if (entry.status == VisitedEntry::QUEUED) {
					if (dist < entry.distance) {
						queue.raise_priority(dist + dist_sq(first_neigh, REAL_START), first_neigh);
						entry.requeue(dist, curr_identifier);
					}
				} else {
					queue.push(std::make_pair(dist + dist_sq(first_neigh, REAL_START), first_neigh));
					entry.queue(dist, curr_identifier);
				}
			}
		}
		if (second_neigh != NULL_IDENFITIER && curr_entry.parent != second_neigh) {
			
			auto& entry = visited[second_neigh];
			if (entry.status != VisitedEntry::VISITED) {
				const double dist = curr_entry.distance + dist_sq(curr_identifier, second_neigh);
				if (entry.status == VisitedEntry::QUEUED) {
					if (dist < entry.distance) {
						queue.raise_priority(dist + dist_sq(second_neigh, REAL_START), second_neigh);
						entry.requeue(dist, curr_identifier);
					}
				} else {
					queue.push(std::make_pair(dist + dist_sq(second_neigh, REAL_START), second_neigh));
					entry.queue(dist, curr_identifier);
				}
			}
		}
		if (third_neigh != NULL_IDENFITIER && curr_entry.parent != third_neigh) {
			auto& entry = visited[third_neigh];
			if (entry.status != VisitedEntry::VISITED) {
				const double dist = curr_entry.distance + dist_sq(curr_identifier, third_neigh);
				if (entry.status == VisitedEntry::QUEUED) {
					if (dist < entry.distance) {
						queue.raise_priority(dist + dist_sq(third_neigh, REAL_START), third_neigh);
						entry.requeue(dist, curr_identifier);
					}
				} else {
					queue.push(std::make_pair(dist + dist_sq(third_neigh, REAL_START), third_neigh));
					entry.queue(dist, curr_identifier);
				}
			}
		}
	}
	if (!found) {
		return { PathfindArguments::MapPathResult::FAIL };
	}
	PathfindArguments::MapPathResult result { PathfindArguments::MapPathResult::SUCCESS };
	std::vector<std::pair<PointLocation::Vertex::Point, PointLocation::Vertex::Point>> portals;

	for (unsigned int current_node = real_start, next_node = visited[current_node].parent; next_node != NULL_IDENFITIER; current_node = std::exchange(next_node, visited[next_node].parent)) {
		portals.push_back(portal(current_node, next_node));
	}
	portals.emplace_back(END, END);
	string_pull(BEGIN, END, portals, result.path);
	
	return result;
}


Pather::Pather(const GameData& data) : map_to_id(), id_to_map(), maps() {
	const nlohmann::json& geo = data["geometry"];
	const nlohmann::json& maps = data["maps"];
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
			for (const nlohmann::json& entry : json_spawns) {
				info_spawns.emplace_back(entry[0].get<double>(), entry[1].get<double>());
			}
			if (data.was_cached) {
				mLogger->info("Registering cached map {} with id {}", key, id);
				this->maps.emplace_back(key);
			} else {
				mLogger->info("Registering map {} with id {}", key, id);
				this->maps.emplace_back(info);
			}
		} else {
			if (data.was_cached) {
				mLogger->info("Skipping cached map {}", key);
			} else {
				mLogger->info("Skipping map {}", key);
			}
		}
	}
};