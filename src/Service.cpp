#include <fmt/os.h>
#include "Pathfinding/Pather.hpp"
#include <memory>

extern "C" void* init(ServiceInfo<PathfindArguments*, long>*info);

Pather* pather;

void* ipc_handler(void* arguments) {
	PathfindArguments* args = (PathfindArguments*)arguments;
	if (args->start_map != args->end_map) {
		return pather->path_doors(args->start_map, args->end_map);
	} else {
		return pather->maps.at(args->start_map).path(PointLocation::Vertex::Point(args->start.x, args->start.y), PointLocation::Vertex::Point(args->end.x, args->end.y));
	};
}

void* init(ServiceInfo<PathfindArguments*, long>* info) {
    GameData* data = info->G;
	pather = new Pather(data);
	return (void*)&ipc_handler;
}