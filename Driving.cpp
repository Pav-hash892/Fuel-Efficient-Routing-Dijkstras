#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <string>
#include <algorithm>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/haversine.hpp>
#include <osmium/osm/location.hpp>

#include "vehicle.h"
int congestionLevel;
struct Edge {
    int to;
    double fuelCost;
    double distance;
};

struct RoadSegment {
    int from, to;
    double distance;
    double speedLimitKph;
    double elevationChange;
    int numLanes;
    double trafficFactor;
};

struct LatLon {
    double lat;
    double lon;
};

std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;
std::vector<RoadSegment> roads;

// --- Distance ---
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    return osmium::geom::haversine::distance(osmium::Location(lon1, lat1), osmium::Location(lon2, lat2)) / 1000.0;
}

// --- Fuel Cost ---
double estimateFuelCost(double distance_km, double avgSpeed_kph, const Vehicle& vehicle,
                        double elevationChange_m = 0, int numLanes = 2, double trafficFactor = 1.0) {
    const double baseEfficiency = 1.60934 /  vehicle.getFuelPerMile();

    double speedPenalty = (avgSpeed_kph > 90) ? (avgSpeed_kph - 90) * 0.05 : 0;
    double elevationPenalty = elevationChange_m * 0.001;
    double laneBonus = std::log2(std::max(numLanes, 1)) * 0.1;
    double congestionMultiplier = 1.0 + (congestionLevel - 5) * 0.05;
    double trafficPenalty = (trafficFactor * congestionMultiplier - 1.0) * 0.5;


    double adjustedEfficiency = baseEfficiency - speedPenalty - elevationPenalty + laneBonus - trafficPenalty;
    adjustedEfficiency = std::clamp(adjustedEfficiency, 5.0, 25.0);

    return distance_km / adjustedEfficiency;
}

// --- OSM Handler ---
class MyHandler : public osmium::handler::Handler {
public:
    void node(const osmium::Node& node) {
        if (node.location().valid()) {
            node_locations[node.id()] = node.location();
        }
    }

    void way(const osmium::Way& way) {
        if (!way.tags().has_key("highway")) return;

        const auto& nodes = way.nodes();
        bool isOneWay = way.tags().has_tag("oneway", "yes") || way.tags().has_tag("junction", "roundabout");

        double speedLimit = 60.0;
        if (const char* speedTag = way.tags()["maxspeed"]) {
            try { speedLimit = std::stod(speedTag); } catch (...) {}
        }

        int numLanes = 2;
        if (const char* lanesTag = way.tags()["lanes"]) {
            try { numLanes = std::stoi(lanesTag); } catch (...) {}
        }

        double trafficFactor = 1.0;
        const char* highwayType = way.tags()["highway"];
        if (highwayType) {
            std::string type{highwayType};
            if (type == "motorway") trafficFactor = 0.9;
            else if (type == "primary") trafficFactor = 1.0;
            else if (type == "secondary") trafficFactor = 1.1;
            else if (type == "residential") trafficFactor = 1.3;
            else trafficFactor = 1.2;
        }

        for (std::size_t i = 1; i < nodes.size(); ++i) {
            auto from = nodes[i - 1].ref();
            auto to = nodes[i].ref();

            if (node_locations.count(from) && node_locations.count(to)) {
                auto loc1 = node_locations[from];
                auto loc2 = node_locations[to];
                double dist = calculateDistance(loc1.lat(), loc1.lon(), loc2.lat(), loc2.lon());

                roads.push_back({static_cast<int>(from), static_cast<int>(to), dist, speedLimit, 0.0, numLanes, trafficFactor});
                if (!isOneWay) {
                    roads.push_back({static_cast<int>(to), static_cast<int>(from), dist, speedLimit, 0.0, numLanes, trafficFactor});
                }
            }
        }
    }
};

// --- Graph Builder ---
std::unordered_map<int, std::vector<Edge>> buildGraph(const std::vector<RoadSegment>& roads, const Vehicle& vehicle) {
    std::unordered_map<int, std::vector<Edge>> graph;
    for (const auto& r : roads) {
        double cost = estimateFuelCost(r.distance, r.speedLimitKph, vehicle,
                                       r.elevationChange, r.numLanes, r.trafficFactor);
        graph[r.from].push_back({r.to, cost, r.distance});
    }
    return graph;
}


// --- Dijkstra ---
struct PathResult {
    std::unordered_map<int, double> fuelCost;
    std::unordered_map<int, double> distance;
    std::unordered_map<int, int> prev;
};

PathResult dijkstraWithDistance(const std::unordered_map<int, std::vector<Edge>>& graph, int source) {
    std::unordered_map<int, double> fuel, dist;
    std::unordered_map<int, int> prev;
    const double INF = std::numeric_limits<double>::infinity();

    for (const auto& [node, _] : graph) {
        fuel[node] = INF;
        dist[node] = INF;
        prev[node] = -1;
    }

    fuel[source] = 0.0;
    dist[source] = 0.0;

    using State = std::tuple<double, double, int>; // fuel, distance, node
    std::priority_queue<State, std::vector<State>, std::greater<>> pq;
    pq.push({0.0, 0.0, source});

    while (!pq.empty()) {
        auto [f, d, u] = pq.top(); pq.pop();
        for (const auto& e : graph.at(u)) {
            double newFuel = f + e.fuelCost;
            double newDist = d + e.distance;
            if (newFuel < fuel[e.to]) {
                fuel[e.to] = newFuel;
                dist[e.to] = newDist;
                prev[e.to] = u;
                pq.push({newFuel, newDist, e.to});
            }
        }
    }

    return {fuel, dist, prev};
}


// --- Path Reconstruction ---
std::vector<int> reconstructPath(int start, int end, const std::unordered_map<int, int>& prev) {
    std::vector<int> path;
    int at = end;

    while (at != -1) {
        path.push_back(at);
        auto it = prev.find(at);
        if (it == prev.end()) {
            std::cerr << " Path reconstruction failed: Node " << at << " has no predecessor.\n";
            return {};
        }
        at = it->second;
    }

    std::reverse(path.begin(), path.end());
    if (path.front() != start) {
        std::cerr << " Path does not originate from the expected start node.\n";
        return {};
    }

    return path;
}

// --- Nearest Node ---
int findNearestNode(double lat, double lon, const std::unordered_map<int, std::vector<Edge>>& graph) {
    double minDist = std::numeric_limits<double>::max();
    int nearestId = -1;
    for (const auto& [id, loc] : node_locations) {
        int nodeId = static_cast<int>(id);
        if (graph.find(nodeId) == graph.end() || !loc.valid()) continue;

        double dist = calculateDistance(lat, lon, loc.lat(), loc.lon());
        if (dist < minDist) {
            minDist = dist;
            nearestId = nodeId;
        }
    }
    return nearestId;
}

// --- Main ---
int main() {
    const char* osmFile = "/Users/saipavankukkadapu/Downloads/155a6623-6d6e-4e83-a1d8-9d6d69bad4ad.osm.pbf";
    osmium::io::Reader reader{osmFile};
    MyHandler handler;
    osmium::apply(reader, handler);
    reader.close();

    std::ifstream file("/Users/saipavankukkadapu/CLionProjects/untitled/input.txt");
    std::string vehicleType;
    double startLat, startLon, endLat, endLon;
    file >> congestionLevel;

    while (file >> vehicleType >> startLat >> startLon >> endLat >> endLon) {
        LatLon start{startLat, startLon};
        LatLon end{endLat, endLon};

        Vehicle vehicle(vehicleType);
        auto graph = buildGraph(roads, vehicle);

        int startNode = findNearestNode(start.lat, start.lon, graph);
        int endNode = findNearestNode(end.lat, end.lon, graph);

        if (startNode == -1 || endNode == -1) {
            std::cerr << " Could not find valid connected nodes for vehicle " << vehicleType << "\n";
            continue;
        }

        auto result = dijkstraWithDistance(graph, startNode);
        if (result.prev.find(endNode) == result.prev.end()) {
            std::cerr << " Destination node is unreachable for vehicle " << vehicleType << "\n";
            continue;
        }

        auto path = reconstructPath(startNode, endNode, result.prev);
        vehicle.setDistanceTravelled(result.distance[endNode]);
        vehicle.setFuelUsed(result.fuelCost[endNode]);

        std::cout << " Vehicle: " << vehicle.getCarType() << "\n";
        std::cout << "  Distance Travelled: " << vehicle.getDistanceTravelled() << " km\n";
        std::cout << "  Fuel Used: " << vehicle.getFuelUsed() << " L\n";
        std::cout << "  Path Nodes: ";
        for (int node : path) std::cout << node << " ";
        std::cout << "\n\n";
    }

    return 0;
}























#include "Driving.h"













