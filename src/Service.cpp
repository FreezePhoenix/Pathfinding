#include <fmt/os.h>
#include "Pathfinding/Pather.hpp"
#include <memory>
#include <iostream>

extern "C" ServiceInfo<PathfindArguments, void*>::HANDLER init(ServiceInfo<PathfindArguments, void>*info);

Pather* pather;

void* ipc_handler(const PathfindArguments& args) {
	std::cout << args.start_map << std::endl;
	if (args.start_map != args.end_map) {
		return pather->path_doors(args.start_map, args.end_map);
	} else {
		auto mappather = pather->maps.find(args.start_map);
		if (mappather != pather->maps.end()) {
			return mappather->second.path({ args.start.x, args.start.y }, { args.end.x, args.end.y });
		}
		return nullptr;
	};
}

void cleanup() {
	delete pather;
	pather = nullptr;
}

ServiceInfo<PathfindArguments, void*>::HANDLER init(ServiceInfo<PathfindArguments, void>* info) {
	info->destructor = cleanup;
	GameData* data = info->G;
	pather = new Pather(data);
	return ipc_handler;
}