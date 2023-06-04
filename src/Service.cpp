#include <fmt/os.h>
#include "Pathfinding/Pather.hpp"
#include "Pathfinding/PriorityFibonacciQueue.hpp"
#include <memory>
#include <iostream>

extern "C" ServiceInfo<PathfindArguments, void*>::HANDLER init(ServiceInfo<PathfindArguments, void>*info);

Pather* pather;

struct N {
    int key;
    int value;
    bool operator==(const N& other) const {
        return other.value == value;
    }
};

template<>
struct std::hash<N>
{
    std::size_t operator()(N const& n) const noexcept
    {
        return n.value;
    }
};

void* ipc_handler(const PathfindArguments& args) {
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
	
	auto path = (PathfindArguments::PathResult*)ipc_handler(PathfindArguments{ PathfindArguments::Point{ 778,-506}, PathfindArguments::Point{ -700,906 }, "main", "main" });
	// std::cout << path->path.size() << std::endl;
	for (PathfindArguments::Point point : path->path) {
		pather->mLogger->info(std::to_string(point.x) + "," + std::to_string(point.y));
	}
	return ipc_handler;
}