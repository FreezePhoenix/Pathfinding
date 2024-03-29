﻿#include <fmt/os.h>
#include "Pathfinding/Pather.hpp"
#include "Pathfinding/PriorityFibonacciQueue.hpp"
#include <memory>
#include <iostream>

extern "C" void init(ServiceInfo& info);

std::unique_ptr<Pather> pather;

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

PathfindArguments::MapPathResult ipc_handler(const PathfindArguments& args) {
	if (args.start_map != args.end_map) {
		pather->mLogger->info("Cross-map pathfinding not supported ({} -> {})", args.start_map, args.end_map);
		return { PathfindArguments::MapPathResult::FAIL };
	} else {
		if (pather->map_to_id.contains(args.start_map)) {
			unsigned int map_id = pather->map_to_id[args.start_map];
			return pather->maps[map_id].path(args.start, args.end);
		}
		return { PathfindArguments::MapPathResult::FAIL };
	};
}

void cleanup() {
	pather->mLogger->info("DESTRUCTING");
	pather.reset(nullptr);
}

void init(ServiceInfo& info) {
	info.set_destructor(std::function(cleanup));
	info.set_handler(std::function(ipc_handler));
	pather = std::make_unique<Pather>(info.G);

	auto path = ipc_handler(PathfindArguments{ PathfindArguments::Point{ 778,-506}, PathfindArguments::Point{ -700,906 }, "main", "main" });
	std::cout << path.path.size() << std::endl;
	for (PathfindArguments::Point point : path.path) {
		pather->mLogger->info(std::to_string(point.x) + "," + std::to_string(point.y));
	}
}