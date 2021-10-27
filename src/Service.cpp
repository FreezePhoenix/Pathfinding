#include <map>
#include <iostream>
#include "Pathfinding/Processor.hpp"
#include "Pathfinding/Service.hpp"
#include <memory>
#include <thread>

// This bit is very important.
extern "C" void* init(ServiceInfo<PathfindArguments*, long>*info);

long ipc_handler(void* arguments) {
	return 0;
}


void* init(ServiceInfo<PathfindArguments*, long>* info) {
	GameData* data = info->G;
	nlohmann::json& geo = data->data->at("geometry");
	for (nlohmann::detail::iter_impl<nlohmann::json> it = geo.begin(); it != geo.end(); it++) {
        if (it.value()["placements"].is_array()) {
            geo[it.key()].erase("placements");
        }
        if (it.value()["x_lines"].is_array()) {
            std::shared_ptr<MapProcessing::MapInfo> info = MapProcessing::parse_map(it.value());
            info->name = it.key();
            nlohmann::json& spawns = data->data->at("maps")[it.key()]["spawns"];
            info->spawns = std::vector<std::pair<double, double>>();
            info->spawns.reserve(spawns.size());
            for (nlohmann::detail::iter_impl<nlohmann::json> spawn_it = spawns.begin(); spawn_it != spawns.end(); spawn_it++) {
                if(spawn_it.value().is_array()) {
                    info->spawns.push_back(std::pair<double, double>(spawn_it.value()[0].get<double>(), spawn_it.value()[1].get<double>()));
                }
            };
            Processor::process(info);
        }
    }
	return (void*)&ipc_handler;
}
