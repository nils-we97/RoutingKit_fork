#include <routingkit/osm_graph_builder.h>
#include <routingkit/osm_profile.h>
#include <routingkit/strongly_connected_component.h>
#include <iostream>
#include <fstream>

#include <vector>
#include <stdint.h>
#include <string>
#include <stdio.h>
#include <memory>

using namespace RoutingKit;
using namespace std;

void log_message(const std::string&msg){
    cout << msg << endl;
}

/// Usage: osm_graph_builder <directory> <PBF file name>
int main(int argc, char*argv[]) {
    string directory;
    string pbf_file;

    if(argc == 3){
        directory = argv[1];
        pbf_file = argv[2];
    } else {
        cout << "Usage: " << argv[0] << "../some/directory/ test.pbf\n" << endl;
        return 1;
    }

    vector<unsigned>first_out;
    vector<unsigned>tail;
    vector<unsigned>head;
    vector<unsigned>travel_time;
    vector<unsigned>geo_distance;
    vector<float>latitude;
    vector<float>longitude;
    vector<unsigned> capacity;
    vector<unsigned> largest_scc;

    {
        auto mapping = load_osm_id_mapping_from_pbf(
            directory + "/" + pbf_file,
            nullptr,
            [&](uint64_t osm_way_id, const TagMap&tags){
                return is_osm_way_used_by_cars(osm_way_id, tags, log_message);
            },
            log_message,
            false
        );

        vector<uint32_t>way_speed(mapping.is_routing_way.population_count()); // in km/h

        auto routing_graph = load_osm_routing_graph_from_pbf(
            directory + "/" + pbf_file,
            mapping,
            [&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&way_tags){
                way_speed[routing_way_id] = get_osm_way_speed(osm_way_id, way_tags, log_message);
                return get_osm_car_direction_category(osm_way_id, way_tags, log_message);
            },
            [&](uint64_t osm_relation_id, const std::vector<OSMRelationMember>&member_list, const TagMap&tags, std::function<void(OSMTurnRestriction)>on_new_restriction){
                return decode_osm_car_turn_restrictions(osm_relation_id, member_list, tags, on_new_restriction, log_message);
            },
            log_message
        );

        mapping = OSMRoutingIDMapping(); // release memory

        first_out = move(routing_graph.first_out);
        tail = invert_inverse_vector(first_out);
        head = move(routing_graph.head);
        latitude = move(routing_graph.latitude);
        longitude = move(routing_graph.longitude);
        geo_distance = move(routing_graph.geo_distance);
        capacity = move(routing_graph.capacity);

        vector<bool> is_largest_scc = compute_largest_strongly_connected_component(first_out, head);
        largest_scc.reserve(is_largest_scc.size());
        for(unsigned i = 0; i < is_largest_scc.size(); i++) {
            largest_scc.push_back(int(is_largest_scc[i]));
        }

        // obtain travel time from geo_distance and way_speed (distance is given in [m])
        travel_time.reserve(geo_distance.size());
        for(unsigned i = 0; i < geo_distance.size(); i++) {
            unsigned tt = (geo_distance[i] * 18) / (way_speed[routing_graph.way[i]] * 5);

            // filter edges with an invalid travel time (should not exceed 24 hours!)
            if(tt > 86400) {
                tt = 86400000;
            } else if(tt == 0) {
                tt = 1;
            } else {
                tt *= 1000;
            }

            travel_time.push_back(tt);
            //travel_time.push_back(geo_distance[i]);
        }

        for(unsigned a=0; a<travel_time.size(); ++a){
            travel_time[a] *= 18;
            travel_time[a] /= way_speed[routing_graph.way[a]];
            travel_time[a] /= 5;
        }

        cout << "Graph properties: " << first_out.size() - 1 << " vertices, " << head.size() << " edges" << endl;
        cout << "Writing results.." << endl;

        std::ofstream out;
        out.open(directory + "/first_out");
        out.write(reinterpret_cast<const char*>(first_out.data()), first_out.size() * sizeof(unsigned));
        out.close();

        out.open(directory + "/head");
        out.write(reinterpret_cast<const char*>(head.data()), head.size() * sizeof(unsigned));
        out.close();

        out.open(directory + "/travel_time");
        out.write(reinterpret_cast<const char*>(travel_time.data()), travel_time.size() * sizeof(unsigned));
        out.close();

        out.open(directory + "/geo_distance");
        out.write(reinterpret_cast<const char*>(geo_distance.data()), geo_distance.size() * sizeof(unsigned));
        out.close();

        out.open(directory + "/capacity");
        out.write(reinterpret_cast<const char*>(capacity.data()), capacity.size() * sizeof(unsigned));
        out.close();

        out.open(directory + "/latitude");
        out.write(reinterpret_cast<const char*>(latitude.data()), latitude.size() * sizeof(float));
        out.close();

        out.open(directory + "/longitude");
        out.write(reinterpret_cast<const char*>(longitude.data()), longitude.size() * sizeof(float));
        out.close();

        out.open(directory + "/largest_scc");
        out.write(reinterpret_cast<const char*>(largest_scc.data()), largest_scc.size() * sizeof(unsigned));
        out.close();
    }
}
