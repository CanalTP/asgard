#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE handle_matrix_test

#include "tile_maker.h"

#include "utils/zmq.h"
#include "asgard/conf.h"
#include "asgard/context.h"
#include "asgard/handler.h"
#include "asgard/metrics.h"
#include "asgard/projector.h"
#include "asgard/request.pb.h"

#include <valhalla/midgard/pointll.h>

#include <boost/test/unit_test.hpp>

using namespace valhalla;

namespace asgard {

namespace config {

// Need the define this otherwise it won't compile
const char* asgard_build_type = "whatever";
const char* project_version = "whatever2";

} // namespace config

const std::string make_string_from_point(const midgard::PointLL& p) {
    return std::string("coord:") +
           std::to_string(p.first) +
           ":" +
           std::to_string(p.second);
}

void add_origin_or_dest_to_request(pbnavitia::LocationContext* location, const std::string& coord) {
    location->set_place(coord);
    location->set_access_duration(0);
}

BOOST_AUTO_TEST_CASE(handle_matrix_test) {
    tile_maker::TileMaker maker;
    maker.make_tile();

    zmq::context_t context(1);
    const Metrics metrics{boost::none};
    const Projector projector{10, 0, 0};

    boost::property_tree::ptree conf;
    conf.put("tile_dir", maker.get_tile_dir());
    valhalla::baldr::GraphReader graph(conf);
    Context c{context, graph, metrics, projector};

    Handler h{c};

    pbnavitia::Request request;
    request.set_requested_api(pbnavitia::street_network_routing_matrix);
    auto* sn_request = request.mutable_sn_routing_matrix();

    auto* request_params = sn_request->mutable_streetnetwork_params();

    add_origin_or_dest_to_request(sn_request->add_origins(),
                                  make_string_from_point(maker.get_all_points().front()));

    for (auto const& p : maker.get_all_points()) {
        add_origin_or_dest_to_request(sn_request->add_destinations(), make_string_from_point(p));
    }

    sn_request->set_mode("walking");
    sn_request->set_max_duration(100000);

    request_params->set_walking_speed(2);

    const auto response = h.handle(request);

    std::vector<unsigned int> expected_times = {0, 111, 444, 667, 359, 568};
    pbnavitia::Response expected_response;
    auto* row = expected_response.mutable_sn_routing_matrix()->add_rows();
    for (auto const& time : expected_times) {
        auto* k = row->add_routing_response();
        k->set_routing_status(pbnavitia::RoutingStatus::reached);
        k->set_duration(time);
    }

    BOOST_CHECK_EQUAL(expected_response.DebugString(), response.DebugString());
}

void check_journey_trivial_direct_path(const pbnavitia::Response& response,
                                       const std::string& origin_uri,
                                       const std::string& destination_uri,
                                       const std::vector<unsigned int>& expected_section_length,
                                       const std::vector<unsigned int>& expected_section_duration) {
    BOOST_ASSERT(response.journeys_size() == 1);

    const auto& journey = response.journeys(0);
    BOOST_CHECK_EQUAL(journey.duration(), 667);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), 0);
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), 667);
    BOOST_CHECK_EQUAL(journey.durations().walking(), 667);
    BOOST_CHECK_EQUAL(journey.durations().walking(), journey.durations().total());
    BOOST_CHECK_EQUAL(journey.distances().walking(), 1334);

    BOOST_ASSERT(journey.nb_sections() == 1);
    const auto& section = journey.sections(0);
    BOOST_CHECK_EQUAL(section.duration(), journey.duration());
    BOOST_CHECK_EQUAL(section.length(), journey.distances().walking());
    BOOST_CHECK_EQUAL(section.origin().uri(), origin_uri);
    BOOST_CHECK_EQUAL(section.destination().uri(), destination_uri);

    const auto& sn = section.street_network();
    BOOST_CHECK_EQUAL(sn.path_items_size(), 2);
    for (int i = 0; i < sn.path_items_size(); ++i) {
        BOOST_CHECK_CLOSE(sn.path_items(i).length(), expected_section_length[i], 0.5f);
        BOOST_CHECK_CLOSE(sn.path_items(i).duration(), expected_section_duration[i], 0.5f);
    }
}

BOOST_AUTO_TEST_CASE(handle_direct_path_trivial_test) {
    tile_maker::TileMaker maker;
    maker.make_tile();

    zmq::context_t context(1);
    const Metrics metrics{boost::none};
    const Projector projector{10, 0, 0};

    boost::property_tree::ptree conf;
    conf.put("tile_dir", maker.get_tile_dir());
    valhalla::baldr::GraphReader graph(conf);
    Context c{context, graph, metrics, projector};

    Handler h{c};

    pbnavitia::Request request;
    request.set_requested_api(pbnavitia::direct_path);
    auto* dp_request = request.mutable_direct_path();

    // Origin is A
    add_origin_or_dest_to_request(dp_request->mutable_origin(),
                                  make_string_from_point(maker.get_all_points().front()));

    // Destination is D
    add_origin_or_dest_to_request(dp_request->mutable_destination(),
                                  make_string_from_point(maker.get_all_points().at(3)));

    auto* sn_params = dp_request->mutable_streetnetwork_params();
    sn_params->set_origin_mode("walking");
    sn_params->set_walking_speed(2);

    // Last section corresponds to the arrival so its length and duration equal 0
    std::vector<unsigned int> expected_section_length = {1334, 0};
    std::vector<unsigned int> expected_section_duration = {667, 0};

    {
        const auto response = h.handle(request);
        check_journey_trivial_direct_path(response, "0.00100;0.00300", "0.01300;0.00300",
                                          expected_section_length, expected_section_duration);
    }

    // We now do the same journey but we swap origin and destination

    // Origin is D
    add_origin_or_dest_to_request(dp_request->mutable_origin(),
                                  make_string_from_point(maker.get_all_points().at(3)));

    // Origin is A
    add_origin_or_dest_to_request(dp_request->mutable_destination(),
                                  make_string_from_point(maker.get_all_points().front()));

    {
        const auto response = h.handle(request);

        // Last section corresponds to the arrival so its length and duration equal 0
        std::vector<unsigned int> reverse_expected_section_length = {1334, 0};
        std::vector<unsigned int> reverse_expected_section_duration = {667, 0};
        check_journey_trivial_direct_path(response, "0.01300;0.00300", "0.00100;0.00300",
                                          reverse_expected_section_length, reverse_expected_section_duration);
    }
}

void check_bss_journey_direct_path(const pbnavitia::Response& response,
                                   const std::vector<float>& expected_section_length,
                                   const std::vector<float>& expected_section_duration) {
    BOOST_ASSERT(response.journeys_size() == 1);

    const auto& journey = response.journeys(0);
    auto total_duration = 1 + std::accumulate(expected_section_duration.begin(), expected_section_duration.end(), 0);
    BOOST_CHECK_EQUAL(journey.duration(), total_duration);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), 0);
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), total_duration);
    BOOST_CHECK_EQUAL(journey.durations().walking(), 277);
    BOOST_CHECK_EQUAL(journey.distances().walking(), 556);
    BOOST_CHECK_EQUAL(journey.durations().bike(), 400);
    BOOST_CHECK_EQUAL(journey.distances().bike(), 1556);

    BOOST_ASSERT(journey.nb_sections() == 5);
    for (int i = 0; i < journey.sections_size(); ++i) {
        BOOST_CHECK_CLOSE(journey.sections(i).length(), expected_section_length[i], 0.5f);
        BOOST_CHECK_CLOSE(journey.sections(i).duration(), expected_section_duration[i], 0.5f);
    }
}

BOOST_AUTO_TEST_CASE(handle_trivial_BSS_test) {
    tile_maker::TileMaker maker;
    maker.make_tile();

    zmq::context_t context(1);
    const Metrics metrics{boost::none};
    const Projector projector{10, 0, 0};

    boost::property_tree::ptree conf;
    conf.put("tile_dir", maker.get_tile_dir());
    valhalla::baldr::GraphReader graph(conf);
    Context c{context, graph, metrics, projector};

    Handler h{c};

    pbnavitia::Request request;
    request.set_requested_api(pbnavitia::direct_path);
    auto* dp_request = request.mutable_direct_path();

    // Origin is A
    add_origin_or_dest_to_request(dp_request->mutable_origin(),
                                  make_string_from_point(maker.a.second));

    // Destination is i
    add_origin_or_dest_to_request(dp_request->mutable_destination(),
                                  make_string_from_point(maker.i.second));

    auto* sn_params = dp_request->mutable_streetnetwork_params();
    sn_params->set_origin_mode("bss");
    sn_params->set_walking_speed(2);
    sn_params->set_bike_speed(4);

    // Last section corresponds to the arrival so its length and duration equal 0
    std::vector<float> expected_section_length = {111, 0, 1556, 0, 445};
    std::vector<float> expected_section_duration = {55, 120, 400, 120, 222};

    {
        const auto response = h.handle(request);
        check_bss_journey_direct_path(response,
                                      expected_section_length, expected_section_duration);
    }
}

BOOST_AUTO_TEST_CASE(handle_trivial_BSS_with_maneuver_duration_test) {
    tile_maker::TileMaker maker;
    maker.make_tile();

    zmq::context_t context(1);
    const Metrics metrics{boost::none};
    const Projector projector{10, 0, 0};

    boost::property_tree::ptree conf;
    conf.put("tile_dir", maker.get_tile_dir());
    valhalla::baldr::GraphReader graph(conf);
    Context c{context, graph, metrics, projector};

    Handler h{c};

    pbnavitia::Request request;
    request.set_requested_api(pbnavitia::direct_path);
    auto* dp_request = request.mutable_direct_path();

    // Origin is A
    add_origin_or_dest_to_request(dp_request->mutable_origin(),
                                  make_string_from_point(maker.a.second));

    // Destination is i
    add_origin_or_dest_to_request(dp_request->mutable_destination(),
                                  make_string_from_point(maker.i.second));

    auto* sn_params = dp_request->mutable_streetnetwork_params();
    sn_params->set_origin_mode("bss");
    sn_params->set_walking_speed(2);
    sn_params->set_bike_speed(4);
    float rent_duration = 42;
    float return_duration = 24;

    sn_params->set_bss_rent_duration(rent_duration);
    sn_params->set_bss_return_duration(return_duration);

    // Last section corresponds to the arrival so its length and duration equal 0
    std::vector<float> expected_section_length = {111, 0, 1556, 0, 445};
    std::vector<float> expected_section_duration = {55, rent_duration, 400, return_duration, 222};

    {
        const auto response = h.handle(request);
        check_bss_journey_direct_path(response,
                                      expected_section_length, expected_section_duration);
    }
}
} // namespace asgard
