#include <fmt/os.h>
#include "Pathfinding/Pather.hpp"
#include <memory>
#include <iostream>

extern "C" void* init(ServiceInfo<PathfindArguments, void>*info);

Pather* pather;

void* ipc_handler(void* arguments) {
	PathfindArguments* args = (PathfindArguments*)arguments;
	std::cout << args->start_map << std::endl;
	if (args->start_map != args->end_map) {
		return pather->path_doors(args->start_map, args->end_map);
	} else {
		auto mappather = pather->maps.find(args->start_map);
		if (mappather != pather->maps.end()) {
			return mappather->second.path(PointLocation::Vertex::Point(args->start.x, args->start.y), PointLocation::Vertex::Point(args->end.x, args->end.y));
		}
		return nullptr;
	};
}



void* init(ServiceInfo<PathfindArguments, void>* info) {
    GameData* data = info->G;
	pather = new Pather(data);
	return (void*)&ipc_handler;
}