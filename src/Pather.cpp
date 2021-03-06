#include <Pathfinding/Pather.hpp>
#include "Pathfinding/Objectifier.hpp"
#include <chrono>



MapPather::MapPather(std::shared_ptr<MapProcessing::MapInfo> info): children(), roots(), neighbhors(), centers() {
	this->mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Pathfinding:MapPather(" + info->name + ")");
	Objectifier* objectifier = new Objectifier(info);
	objectifier->init(true);
	objectifier->run();

	// Really, this method is pretty stupidly complicated, and it changes *a lot*
	// As such, I'm not going to document it. It's intirely internal, and should not be used by a user.
	std::shared_ptr<triangulateio> input = TriangleManipulator::create_instance();
	ShapeManipulator::from_list(objectifier->lines, input);
	if (input->numberofsegments < 3) {
		return;
	}
	
	std::shared_ptr<triangulateio> output = TriangleManipulator::create_instance();
	std::shared_ptr<triangulateio> voutput = TriangleManipulator::create_instance();
	std::shared_ptr<triangulateio> trimmed_input = TriangleManipulator::create_instance();
	
	int num_holes = input->numberofholes = objectifier->holes.size();
	input->holelist = trimalloc<double>(num_holes * 2);
	double* hole_ptr = input->holelist.get();
	for (const MapProcessing::Point& hole : objectifier->holes) {
		*hole_ptr++ = hole.x;
		*hole_ptr++ = hole.y;
	}
	triangulate("pznejQv", input, output, voutput);
	children.resize(output->numberoftriangles);
	roots.resize(output->numberoftriangles);
	centers.reserve(output->numberoftriangles);
	std::shared_ptr<triangulateio> temp = TriangleManipulator::create_instance();
	TriangleManipulator::filter_edges(voutput, temp, [](int p1, int p2, REAL norm1, REAL norm2) {
		return p2 != -1;
	});
	std::shared_ptr<int> subdomains = trimalloc<int>(output->numberoftriangles);
	std::map<int, int> mapped_edges = std::map<int, int>();
	for (unsigned int i = 0; i < output->numberoftriangles; i++) {
		subdomains.get()[i] = 2;
		mapped_edges.insert_or_assign(i, -1);
	}
	int* edge_ptr = output->edgelist.get();
	int* neigh_ptr = output->neighborlist.get();
	int* edge_marker_ptr = output->edgemarkerlist.get();
	int* v_edge_list = voutput->edgelist.get();
	unsigned int* tri_ptr = output->trianglelist.get();
	double* point_ptr = output->pointlist.get();
	
	std::queue<std::pair<int, int>> queue = std::queue<std::pair<int, int>>();
	for (int i = 0; i < output->numberoftriangles; i++) {
		int first_tri = neigh_ptr[i * 3];
		int second_tri = neigh_ptr[i * 3 + 1];
		int third_tri = neigh_ptr[i * 3 + 2];
		if (first_tri == -1) {
			subdomains.get()[i] -= 1;
			if (subdomains.get()[i] == 0) {
				const int&& remaining = neigh_ptr[i * 3] + neigh_ptr[i * 3 + 1] + neigh_ptr[i * 3 + 2] - first_tri - mapped_edges.at(i);
				queue.emplace(remaining, i);
			} else {
				mapped_edges.insert_or_assign(i, first_tri);
			}
		} else {
			children[first_tri].insert(i);
		}
		if (second_tri == -1) {
			subdomains.get()[i] -= 1;
			if (subdomains.get()[i] == 0) {
				const int&& remaining = neigh_ptr[i * 3] + neigh_ptr[i * 3 + 1] + neigh_ptr[i * 3 + 2] - second_tri - mapped_edges.at(i);
				queue.emplace(remaining, i);
			} else {
				mapped_edges.insert_or_assign(i, second_tri);
			}
		} else {
			children[second_tri].insert(i);
		}
		if (third_tri == -1) {
			subdomains.get()[i] -= 1;
			if (subdomains.get()[i] == 0) {
				const int&& remaining = neigh_ptr[i * 3] + neigh_ptr[i * 3 + 1] + neigh_ptr[i * 3 + 2] - third_tri - mapped_edges.at(i);
				queue.emplace(remaining, i);
			} else {
				mapped_edges.insert_or_assign(i, third_tri);
			}
		} else {
			children[third_tri].insert(i);
		}
	}
	
	std::shared_ptr<triangulateio> temp2 = TriangleManipulator::create_instance();
	unsigned int numb_edges = 0;
	std::vector<int> edges = std::vector<int>();
	int* subdomain_ptr = subdomains.get();
	while (queue.size() > 0) {
		int first, second;
		std::tie(first, second) = queue.front();
		numb_edges++;
		edges.push_back(first);
		edges.push_back(second);
		children[first].insert(second);
		children[first].insert(children[second].begin(), children[second].end());
		if (subdomains.get()[first] > 0) {
			subdomains.get()[first] -= 1;
			if (subdomains.get()[first] == 0) {
				int remaining = neigh_ptr[first * 3] + neigh_ptr[first * 3 + 1] + neigh_ptr[first * 3 + 2] - second - mapped_edges.at(first);
				queue.emplace(remaining, first);
			} else {
				mapped_edges.insert_or_assign(first, second);
			}
		}
		queue.pop();
	}
	for (unsigned int i = 0; i < output->numberoftriangles; i++) {
		const unsigned int first = tri_ptr[i * 3],
			second = tri_ptr[i * 3 + 1],
			third = tri_ptr[i * 3 + 2];
		const double new_x = (point_ptr[first * 2] + point_ptr[second * 2] + point_ptr[third * 2]) / 3;
		const double new_y = (point_ptr[first * 2 + 1] + point_ptr[second * 2 + 1] + point_ptr[third * 2 + 1]) / 3;
		centers.emplace_back(new_x, new_y);
		int subdomain = subdomains.get()[i];
		if (subdomain != 0) {
			roots[i] = i;
			int first_neigh = -1, second_neigh = -1, third_neigh = -1;
			if (neigh_ptr[i * 3] != -1 && subdomains.get()[neigh_ptr[i * 3]] != 0) {
				first_neigh = neigh_ptr[i * 3];
			}
			if (neigh_ptr[i * 3 + 1] != -1 && subdomains.get()[neigh_ptr[i * 3 + 1]] != 0) {
				second_neigh = neigh_ptr[i * 3 + 1];
			}
			if (neigh_ptr[i * 3 + 2] != -1 && subdomains.get()[neigh_ptr[i * 3 + 2]] != 0) {
				third_neigh = neigh_ptr[i * 3 + 2];
			}
			neighbhors.emplace(i, std::tuple<int, int, int>(first_neigh, second_neigh, third_neigh));
			for (unsigned int child : children[i]) {
				if (subdomains.get()[child] == 0) {
					roots[child] = i;
				}
			}
		}
	}
	for (unsigned int i = 0; i < voutput->numberofpoints; i++) {
		const unsigned int first = tri_ptr[i * 3],
			second = tri_ptr[i * 3 + 1],
			third = tri_ptr[i * 3 + 2];
		const double new_x = (point_ptr[first * 2] + point_ptr[second * 2] + point_ptr[third * 2]) / 3;
		const double new_y = (point_ptr[first * 2 + 1] + point_ptr[second * 2 + 1] + point_ptr[third * 2 + 1]) / 3;
		voutput->pointlist.get()[i * 2] = new_x;
		voutput->pointlist.get()[i * 2 + 1] = new_y;
	}
	temp2->numberofedges = numb_edges;
	temp2->edgelist = std::shared_ptr<int>(edges.data(), [](void*) {});
	output->numberofsubdomains = 3;
	output->subdomainlist = subdomains;
	this->graph = output;
	this->graph.process();
	this->graph.map_triangles(output);
	TriangleManipulator::write_part_file("Maps/" + info->name + ".part", output);
	TriangleManipulator::write_poly_file("Maps/" + info->name + ".poly", input);
	TriangleManipulator::write_edge_file("Maps/" + info->name + ".v.edge", temp2);
	TriangleManipulator::write_node_file("Maps/" + info->name + ".v.node", voutput);
	TriangleManipulator::write_node_file("Maps/" + info->name + ".node", output);
	TriangleManipulator::write_ele_file("Maps/" + info->name + ".ele", output);
	TriangleManipulator::write_neigh_file("Maps/" + info->name + ".neigh", output);
	TriangleManipulator::write_edge_file("Maps/" + info->name + ".edge", output);
	this->write_to_file(info->name);
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
	for (std::pair<int, std::tuple<int, int, int>> _pair : neighbhors) {
		writer.write(_pair.first);
		writer.write(&_pair.second);
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
		const auto first = reader.read<int>();
		const auto second = reader.read<std::tuple<int, int, int>>();
		neighbhors.emplace(first, second);
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
	unsigned int real_start = roots[start];
	unsigned int real_end = roots[end];
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
	unsigned int current_node = real_start;
	while (PARENTS.at(current_node) != (unsigned int) -1) {
		result->path.emplace_back(centers[current_node]);
		current_node = PARENTS.at(current_node);
	}
	result->path.emplace_back(centers[current_node]);
    auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	mLogger->info("Pathfinding took: " + std::to_string(ms_double.count()) + "ms");
	return result;
}


Pather::Pather(GameData* data) : data(data), maps(), map_to_id(), id_to_map(), door_map_in(), door_map_out(), doors_in_map(), door_in(), door_out(), door_spawn_out(), door_spawn_in() {
	if (data->was_cached && false) {
		nlohmann::json& geo = data->data->at("geometry");
		for (nlohmann::detail::iter_impl<nlohmann::json> it = geo.begin(); it != geo.end(); it++) {
			if (it.value()["x_lines"].is_array() && it.value()["x_lines"].size() > 1) {
				this->maps.emplace(it.key(), MapPather(it.key()));
			}
		}
	} else {
		nlohmann::json& geo = data->data->at("geometry");
		for (nlohmann::detail::iter_impl<nlohmann::json> it = geo.begin(); it != geo.end(); it++) {
			if (it.value()["placements"].is_array()) {
				geo[it.key()].erase("placements");
			}
			if (it.value()["x_lines"].is_array()) {
				int id = map_to_id.size();
				map_to_id.emplace(it.key(), id);
				id_to_map.emplace_back(it.key());
				mLogger->info("Registering map " + it.key() + " with id " + std::to_string(id));
				// MapPather
				std::shared_ptr<MapProcessing::MapInfo> info = MapProcessing::parse_map(it.value());
				info->name = it.key();
				nlohmann::json& spawns = data->data->at("maps")[it.key()]["spawns"];
				info->spawns = std::vector<std::pair<double, double>>();
				info->spawns.reserve(spawns.size());
				for (nlohmann::detail::iter_impl<nlohmann::json> spawn_it = spawns.begin(); spawn_it != spawns.end(); spawn_it++) {
					if (spawn_it.value().is_array()) {
						info->spawns.push_back(std::pair<double, double>(spawn_it.value()[0].get<double>(), spawn_it.value()[1].get<double>()));
					}
				};
				this->maps.emplace(info->name, info);
				
			} else {
				mLogger->info("Skipping map " + it.key());
			}
		}
		// A second pass to generate door paths.
		doors_in_map.resize(maps.size());
		for (nlohmann::detail::iter_impl<nlohmann::json> it = geo.begin(); it != geo.end(); it++) {
			if (map_to_id.contains(it.key())) {
				int map_id = map_to_id.at(it.key());
				nlohmann::json& spawns = data->data->at("maps")[it.key()]["spawns"];
				nlohmann::json& doors = data->data->at("maps")[it.key()]["doors"];
				if (doors.is_array()) {
					int index = 0;
					// doors_in_map[map_id] = std::vector<int>();
					doors_in_map[map_id].reserve(doors.size());
					for (nlohmann::detail::iter_impl<nlohmann::json> door_it = doors.begin(); door_it != doors.end(); door_it++, index++) {
						nlohmann::json& door = door_it.value();
						std::string out_map = door.at(4).get<std::string>();
						if (map_to_id.contains(out_map)) {
							int id = NEXT_DOOR_ID++;
							doors_in_map[map_id].emplace_back(id);
							int out_map_id = map_to_id.at(out_map);
							nlohmann::json& in_spawn = spawns.at(door.at(6).get<int>());
							nlohmann::json& out_spawn = data->data->at("maps").at(out_map).at("spawns").at(door.at(5).get<int>());
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
							mLogger->warn("Door " + std::to_string(index) + " in map " + it.key() + " leads to invalid map " + out_map);
						}
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