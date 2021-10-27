#include "TriangleManipulator/TriangleManipulator.hpp"
#include "TriangleManipulator/ShapeManipulator.hpp"
#include "albot/MapProcessing/MapProcessing.hpp"
#include "Pathfinding/Writer.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

static std::shared_ptr<spdlog::logger> mLogger = spdlog::stdout_color_mt<spdlog::async_factory>("Writer");

Writer::Writer(Objectifier& objectifier): objectifier(objectifier) {}
std::mutex M;
void Writer::write() {
    // Really, this method is pretty stupidly complicated, and it changes *a lot*
    // As such, I'm not going to document it. It's intirely internal, and should not be used by a user.
    
    std::shared_ptr<MapProcessing::MapInfo> info = this->objectifier.info;
    std::shared_ptr<triangulateio> input = TriangleManipulator::create_instance();
    std::shared_ptr<triangulateio> output = TriangleManipulator::create_instance();
    std::shared_ptr<triangulateio> voutput = TriangleManipulator::create_instance();
    std::shared_ptr<triangulateio> trimmed_input = TriangleManipulator::create_instance();
    int num_holes = 0;
    std::shared_ptr<std::vector<double>> holes = std::shared_ptr<std::vector<double>>(new std::vector<double>());
    ShapeManipulator::from_list(this->objectifier.lines, input);
    TriangleManipulator::write_poly_file("Maps/" + info->name + ".poly", input);
    if (input->numberofsegments < 3) {
        mLogger->warn("Problem: {}", info->name);
        return;
    }
    mLogger->info("Map: {}", info->name);
    bool first = true;
    int index = 0;
    for (std::shared_ptr<std::vector<MapProcessing::Line>> obj : this->objectifier.objects) {
        if (first) {
            first = false;
            std::shared_ptr<triangulateio> objecto = TriangleManipulator::create_instance();
            ShapeManipulator::from_list(*obj, objecto);
            objecto->numberofholes = info->spawns.size();;
            objecto->holelist = trimalloc<REAL>(objecto->numberofholes * 2);
            const std::size_t max = info->spawns.size();
            const std::pair<double, double>* ptr = info->spawns.data();
            REAL* hole_ptr = objecto->holelist.get();
            for (std::size_t i = 0; i < max; i++) {
                hole_ptr[i * 2] = ptr[i].first;
                hole_ptr[i * 2 + 1] = ptr[i].second;
            }
            std::shared_ptr<triangulateio> output = TriangleManipulator::create_instance();
            TriangleManipulator::write_poly_file("Maps/" + info->name + ".object." + std::to_string(++index) + ".poly", objecto);
            triangulate("pzDQ", objecto, output, nullptr);
            // output->numberofholes = 0;
            std::shared_ptr<std::vector<double>> new_holes = ShapeManipulator::find_points_inside(output);
            num_holes += new_holes->size() / 2;
            holes->insert(holes->end(), new_holes->begin(), new_holes->end());
        } else {
            std::shared_ptr<triangulateio> triangle_object = TriangleManipulator::create_instance();
            ShapeManipulator::from_list(*obj, triangle_object);
            std::shared_ptr<std::vector<double>> new_holes = ShapeManipulator::find_points_inside(*obj);
            triangle_object->holelist = std::shared_ptr<double>(new_holes->data(), [](void*) {});
            triangle_object->numberofholes = new_holes->size() / 2;
            TriangleManipulator::write_poly_file("Maps/" + info->name + ".object." + std::to_string(++index) + ".poly", triangle_object);
            num_holes += new_holes->size() / 2;
            holes->insert(holes->end(), new_holes->cbegin(), new_holes->cend());
        }
    }
    input->numberofholes = num_holes;
    input->holelist = std::shared_ptr<double>(holes->data(), [](void*) {});
    triangulate("pznejQYv", input, output, voutput);
    std::shared_ptr<triangulateio> temp = TriangleManipulator::create_instance();
    TriangleManipulator::filter_edges(voutput, temp, [](int p1, int p2, REAL norm1, REAL norm2) {
        return p2 != -1;
    });
    std::shared_ptr<int> subdomains = trimalloc<int>(output->numberoftriangles);
    std::map<int, int> mapped_edges = std::map<int, int>();
    for (int i = 0; i < output->numberoftriangles; i++) {
        subdomains.get()[i] = 2;
        mapped_edges.insert_or_assign(i, -1);
    }
    int* edge_ptr = output->edgelist.get();
    int* neigh_ptr = output->neighborlist.get();
    int* edge_marker_ptr = output->edgemarkerlist.get();
    int* v_edge_list = voutput->edgelist.get();
    unsigned int* tri_ptr = output->trianglelist.get();
    double* point_ptr = output->pointlist.get();
    std::vector<std::pair<int, int>> queue = std::vector<std::pair<int, int>>();
    for (int i = 0; i < output->numberofedges; i++) {
        if (edge_marker_ptr[i] == 1) {
            int first_tri = v_edge_list[i * 2];
            int second_tri = v_edge_list[i * 2 + 1];
            if (first_tri != -1) {
                subdomains.get()[first_tri] -= 1;
                // 1 edge available
                if (subdomains.get()[first_tri] == 0) {
                    int remaining = neigh_ptr[first_tri * 3] + neigh_ptr[first_tri * 3 + 1] + neigh_ptr[first_tri * 3 + 2] - second_tri - mapped_edges.at(first_tri);
                    queue.emplace_back(remaining, first_tri);
                } else {
                    mapped_edges.insert_or_assign(first_tri, second_tri);// = second_tri;
                }
            }
            if (second_tri != -1) {
                subdomains.get()[second_tri] -= 1;
                if (subdomains.get()[second_tri] == 0) {
                    int remaining = neigh_ptr[second_tri * 3] + neigh_ptr[second_tri * 3 + 1] + neigh_ptr[second_tri * 3 + 2] - first_tri - mapped_edges.at(second_tri);
                    queue.emplace_back(remaining, second_tri);
                } else {
                    mapped_edges.insert_or_assign(second_tri, first_tri);
                }
            }
        }
    }
    std::shared_ptr<triangulateio> temp2 = TriangleManipulator::create_instance();
    unsigned int numb_edges = 0;
    std::vector<int> edges = std::vector<int>();
    int* subdomain_ptr = subdomains.get();
    while (queue.size() > 0) {
        std::vector<std::pair<int, int>>::iterator item = queue.begin();
        int first, second;
        std::tie(first, second) = *item;
        numb_edges++;
        edges.push_back(first);
        edges.push_back(second);
        int& subdomain = subdomain_ptr[first];
        if (subdomains.get()[first] > 0) {
            subdomains.get()[first] -= 1;
            if (subdomains.get()[first] == 0) {
                int remaining = neigh_ptr[first * 3] + neigh_ptr[first * 3 + 1] + neigh_ptr[first * 3 + 2] - second;
                remaining -= mapped_edges.at(first);
                queue.erase(item);
                queue.emplace_back(remaining, first);
            } else {
                mapped_edges.insert_or_assign(first, second);
                queue.erase(item);
            }
        } else {
            queue.erase(item);
        }
    }
    for (int i = 0; i < voutput->numberofpoints; i++) {
        unsigned first, second, third;
        first = tri_ptr[i * 3];
        second = tri_ptr[i * 3 + 1];
        third = tri_ptr[i * 3 + 2];
        double new_x = (point_ptr[first * 2] + point_ptr[second * 2] + point_ptr[third * 2]) / 3;
        double new_y = (point_ptr[first * 2 + 1] + point_ptr[second * 2 + 1] + point_ptr[third * 2 + 1]) / 3;
        voutput->pointlist.get()[i * 2] = new_x;
        voutput->pointlist.get()[i * 2 + 1] = new_y;
    }
    temp2->numberofedges = numb_edges;
    temp2->edgelist = std::shared_ptr<int>(edges.data(), [](void*) {});
    output->numberofsubdomains = 3;
    output->subdomainlist = subdomains;
    output->numberofholes = 0;
    TriangleManipulator::write_part_file("Maps/" + info->name + ".part", output);
    TriangleManipulator::write_poly_file("Maps/" + info->name + ".poly", output);
    TriangleManipulator::write_edge_file("Maps/" + info->name + ".v.edge", temp2);
    TriangleManipulator::write_node_file("Maps/" + info->name + ".v.node", voutput);
    TriangleManipulator::write_node_file("Maps/" + info->name + ".node", output);
    TriangleManipulator::write_ele_file("Maps/" + info->name + ".ele", output);
    TriangleManipulator::write_neigh_file("Maps/" + info->name + ".neigh", output);
    TriangleManipulator::write_edge_file("Maps/" + info->name + ".edge", output);
}