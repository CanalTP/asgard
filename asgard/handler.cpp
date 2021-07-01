// Copyright 2017-2018, CanalTP and/or its affiliates. All rights reserved.
//
// LICENCE: This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU Affero General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public
// License along with this program. If not, see
// <http://www.gnu.org/licenses/>.

#include "asgard/handler.h"
#include "utils/coord_parser.h"
#include "asgard/context.h"
#include "asgard/direct_path_response_builder.h"
#include "asgard/metrics.h"
#include "asgard/projector.h"
#include "asgard/request.pb.h"
#include "asgard/util.h"

#include <valhalla/midgard/pointll.h>
#include <valhalla/odin/directionsbuilder.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/triplegbuilder.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/join.hpp>

#include <ctime>
#include <numeric>
#include <utility>

using namespace valhalla;

namespace asgard {

using ValhallaLocations = google::protobuf::RepeatedPtrField<valhalla::Location>;
using ProjectedLocations = std::unordered_map<midgard::PointLL, valhalla::baldr::PathLocation>;

// The max size of matrix in jormungandr is 5000 so far...
constexpr size_t MAX_MASK_SIZE = 10000;
using ProjectionFailedMask = std::bitset<MAX_MASK_SIZE>;

static const std::unordered_map<std::string, float> TIMECOST_DIVISOR = {{"walking", thor::kTimeDistCostThresholdPedestrianDivisor},
                                                                        {"bike", thor::kTimeDistCostThresholdBicycleDivisor},
                                                                        {"car", thor::kTimeDistCostThresholdAutoDivisor},
                                                                        {"taxi", thor::kTimeDistCostThresholdAutoDivisor}};

// MAX_MATRIX_Distance value set by Valhalla
// in meters
static const std::unordered_map<std::string, float> MAX_MATRIX_DISTANCE = {{"walking", 200000},
                                                                           {"bike", 200000},
                                                                           {"car", 500000},
                                                                           {"taxi", 500000}};

// Default value set by Navitia
// in m/s
static const std::unordered_map<std::string, float> MAX_SPEED = {{"walking", 4},
                                                                 {"bike", 15},
                                                                 {"car", 50},
                                                                 {"taxi", 50}};

namespace {

pbnavitia::Response make_error_response(pbnavitia::Error_error_id err_id, const std::string& err_msg) {
    pbnavitia::Response error_response;
    error_response.set_response_type(pbnavitia::NO_SOLUTION);
    error_response.mutable_error()->set_id(err_id);
    error_response.mutable_error()->set_message(err_msg);
    LOG_ERROR(err_msg + " No solution found !");
    return error_response;
}

float get_distance(const std::string& mode, float duration) {
    if (duration < 0) {
        throw std::runtime_error("Matrix max duration is negative");
    }
    float max_distance = duration * TIMECOST_DIVISOR.at(mode);
    if (max_distance > MAX_MATRIX_DISTANCE.at(mode)) {
        LOG_ERROR(std::string("Matrix distance for mode ") + mode + " is too large, we have to clamp it");
        return MAX_MATRIX_DISTANCE.at(mode);
    }
    return max_distance;
}

void clamp_speed(ModeCostingArgs& args) {
    for (const auto& mode : {"walking", "bike", "car", "taxi"}) {
        if (args.speeds[util::convert_navitia_to_valhalla_costing(mode)] > MAX_SPEED.at(mode)) {
            LOG_ERROR(std::string("Speed for mode ") + mode + " is too large, we have to clamp it");
            args.speeds[util::convert_navitia_to_valhalla_costing(mode)] = MAX_SPEED.at(mode);
        }
    }
}
ModeCostingArgs
make_modecosting_args(const pbnavitia::DirectPathRequest& request) {
    ModeCostingArgs args{};

    auto const& request_params = request.streetnetwork_params();
    args.mode = request.streetnetwork_params().origin_mode();

    args.speeds[util::convert_navitia_to_valhalla_costing("walking")] = request_params.walking_speed();
    args.speeds[util::convert_navitia_to_valhalla_costing("bike")] = request_params.bike_speed();
    args.speeds[util::convert_navitia_to_valhalla_costing("car")] = request_params.car_speed();
    args.speeds[util::convert_navitia_to_valhalla_costing("taxi")] = request_params.car_no_park_speed();

    clamp_speed(args);

    args.bss_rent_duration = request_params.bss_rent_duration();
    args.bss_rent_penalty = request_params.bss_rent_penalty();
    args.bss_return_duration = request_params.bss_return_duration();
    args.bss_return_penalty = request_params.bss_return_penalty();

    return args;
}

ModeCostingArgs
make_modecosting_args(const pbnavitia::StreetNetworkRoutingMatrixRequest& request) {
    ModeCostingArgs args;

    args.mode = request.mode();

    auto const& request_params = request.streetnetwork_params();

    args.speeds[util::convert_navitia_to_valhalla_costing("walking")] = request_params.walking_speed();
    args.speeds[util::convert_navitia_to_valhalla_costing("bike")] = request_params.bike_speed();
    args.speeds[util::convert_navitia_to_valhalla_costing("car")] = request_params.car_speed();
    args.speeds[util::convert_navitia_to_valhalla_costing("taxi")] = request_params.car_no_park_speed();

    clamp_speed(args);

    args.bss_rent_duration = request_params.bss_rent_duration();
    args.bss_rent_penalty = request_params.bss_rent_penalty();
    args.bss_return_duration = request_params.bss_return_duration();
    args.bss_return_penalty = request_params.bss_return_penalty();

    return args;
}

std::pair<ValhallaLocations, ProjectionFailedMask>
make_valhalla_locations_from_projected_locations(const std::vector<midgard::PointLL>& navitia_locations,
                                                 const ProjectedLocations& projected_locations,
                                                 valhalla::baldr::GraphReader& graph) {
    ValhallaLocations valhalla_locations;
    // This mask is used to remember the index of navitia locations whose projection has failed
    // 0 means projection OK, 1 means projection KO
    ProjectionFailedMask projection_failed_mask;

    size_t source_idx = -1;
    for (const auto& l : navitia_locations) {
        ++source_idx;
        auto it = projected_locations.find(l);
        if (it == projected_locations.end()) {
            projection_failed_mask.set(source_idx);
            LOG_ERROR("Cannot project coord: " + std::to_string(l.lng()) + ";" + std::to_string(l.lat()));
            continue;
        }
        baldr::PathLocation::toPBF(it->second, valhalla_locations.Add(), graph);
    }

    return std::make_pair(std::move(valhalla_locations), projection_failed_mask);
}

template<typename T>
struct Finally {
    T t;
    explicit Finally(T t) : t(t){};
    Finally() = delete;
    Finally(Finally&& f) noexcept = default;
    Finally(const Finally&) = delete;
    Finally& operator=(const Finally&) = delete;
    Finally& operator=(Finally&&) = delete;
    ~Finally() {
        try {
            t();
        } catch (...) {
            LOG_ERROR("Error occurred when exectuing Finally");
        }
    };
};

template<typename T>
Finally<T> make_finally(T&& t) {
    return Finally<T>{std::forward<T>(t)};
}

} // namespace

Handler::Handler(const Context& context) : graph(context.graph),
                                           metrics(context.metrics),
                                           projector(context.projector) {
}

pbnavitia::Response Handler::handle(const pbnavitia::Request& request) {
    // RAII to clear all
    auto f = make_finally([this]() { clear(); });

    try {
        switch (request.requested_api()) {
        case pbnavitia::street_network_routing_matrix: return handle_matrix(request);
        case pbnavitia::direct_path: return handle_direct_path(request);
        default:
            LOG_ERROR("wrong request: aborting");
            return pbnavitia::Response();
        }
    } catch (const std::exception& e) {
        return make_error_response(pbnavitia::Error::internal_error, e.what());
    }
}

namespace pt = boost::posix_time;
pbnavitia::Response Handler::handle_matrix(const pbnavitia::Request& request) {
    pt::ptime start = pt::microsec_clock::universal_time();
    const std::string mode = request.sn_routing_matrix().mode();
    LOG_INFO("Processing matrix request " +
             std::to_string(request.sn_routing_matrix().origins_size()) + "x" +
             std::to_string(request.sn_routing_matrix().destinations_size()) +
             " with mode " + mode);

    const auto navitia_sources = util::convert_locations_to_pointLL(request.sn_routing_matrix().origins());
    const auto navitia_targets = util::convert_locations_to_pointLL(request.sn_routing_matrix().destinations());

    mode_costing.update_costing(make_modecosting_args(request.sn_routing_matrix()));

    const auto costing = mode_costing.get_costing_for_mode(mode);

    // We use the cache only when there are more than one element in the sources/targets, so the cache will keep only stop_points coord
    bool use_cache = (navitia_sources.size() > 1);

    const auto projected_sources_locations = projector(begin(navitia_sources), end(navitia_sources), graph, mode, costing, use_cache);
    if (projected_sources_locations.empty()) {
        LOG_ERROR("All sources projections failed!");
        return make_error_response(pbnavitia::Error::no_origin, "origins projection failed!");
    }

    use_cache = (navitia_targets.size() > 1);
    const auto projected_targets_locations = projector(begin(navitia_targets), end(navitia_targets), graph, mode, costing, use_cache);
    if (projected_targets_locations.empty()) {
        LOG_ERROR("All targets projections failed!");
        return make_error_response(pbnavitia::Error::no_destination, "destinations projection failed!");
    }

    ValhallaLocations valhalla_location_sources;
    ProjectionFailedMask projection_mask_sources;
    std::tie(valhalla_location_sources, projection_mask_sources) = make_valhalla_locations_from_projected_locations(navitia_sources, projected_sources_locations, graph);

    ValhallaLocations valhalla_location_targets;
    ProjectionFailedMask projection_mask_targets;
    std::tie(valhalla_location_targets, projection_mask_targets) = make_valhalla_locations_from_projected_locations(navitia_targets, projected_targets_locations, graph);

    LOG_INFO(std::to_string(projection_mask_sources.count()) + " origin(s) projection failed " +
             std::to_string(projection_mask_targets.count()) + " target(s) projection failed");

    std::vector<valhalla::thor::TimeDistance> res;
    if (mode == "bss") {
        res = bss_matrix.SourceToTarget(valhalla_location_sources,
                                        valhalla_location_targets,
                                        graph,
                                        mode_costing.get_costing(),
                                        util::convert_navitia_to_valhalla_mode(mode),
                                        get_distance(mode, request.sn_routing_matrix().max_duration()));

    } else {
        res = matrix.SourceToTarget(valhalla_location_sources,
                                    valhalla_location_targets,
                                    graph,
                                    mode_costing.get_costing(),
                                    util::convert_navitia_to_valhalla_mode(mode),
                                    get_distance(mode, request.sn_routing_matrix().max_duration()));
    }

    pbnavitia::Response response;
    int nb_unreached = 0;
    //in fact jormun don't want a real matrix, only a vector of solution :(
    auto* row = response.mutable_sn_routing_matrix()->add_rows();
    assert(res.size() == valhalla_location_sources.size() * valhalla_location_targets.size());

    const auto& failed_projection_mask = (navitia_sources.size() == 1) ? projection_mask_targets : projection_mask_sources;
    size_t elt_idx = -1;
    size_t resp_row_size = navitia_sources.size() == 1 ? navitia_targets.size() : navitia_sources.size();
    assert(resp_row_size == failed_projection_mask.count() + res.size());

    auto res_it = res.cbegin();
    while (++elt_idx < resp_row_size) {
        auto* k = row->add_routing_response();
        if (failed_projection_mask[elt_idx]) {
            k->set_duration(-1);
            k->set_routing_status(pbnavitia::RoutingStatus::unreached);
            ++nb_unreached;
        } else if (res_it != res.cend()) {
            k->set_duration(res_it->time);
            if (res_it->time == thor::kMaxCost ||
                res_it->time > uint32_t(request.sn_routing_matrix().max_duration())) {
                k->set_routing_status(pbnavitia::RoutingStatus::unreached);
                ++nb_unreached;
            } else {
                k->set_routing_status(pbnavitia::RoutingStatus::reached);
            }
            ++res_it;
        }
    }

    LOG_INFO("Matrix Request done with " + std::to_string(nb_unreached) + " unreached");

    const auto duration = pt::microsec_clock::universal_time() - start;
    metrics.observe_handle_matrix(mode, duration.total_milliseconds() / 1000.0);
    for (auto const& mode : {"walking", "bike", "car"}) {
        metrics.observe_nb_cache_miss(mode, projector.get_nb_cache_miss(mode), projector.get_nb_cache_calls(mode));
        metrics.observe_cache_size(mode, projector.get_current_cache_size(mode));
    }
    return response;
}

// TODO: Since there are more and more algorithms appearing and developped over different usages,
//       we are supposed to enrich this function as what's done here:
//       https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L273
thor::PathAlgorithm& Handler::get_path_algorithm(const valhalla::Location& origin,
                                                 const valhalla::Location& destination,
                                                 const std::string& mode) {
    if (mode == "bss") {
        return bss_astar;
    }

    // Use A* if any origin and destination edges are the same or are connected - otherwise
    // use bidirectional A*. Bidirectional A* does not handle trivial cases with oneways and
    // has issues when cost of origin or destination edge is high (needs a high threshold to
    // find the proper connection).
    for (auto& edge1 : origin.path_edges()) {
        for (auto& edge2 : destination.path_edges()) {
            if (edge1.graph_id() == edge2.graph_id() ||
                graph.AreEdgesConnected(GraphId(edge1.graph_id()), GraphId(edge2.graph_id()))) {
                return timedep_forward;
            }
        }
    }
    return bda;
}

pbnavitia::Response Handler::handle_direct_path(const pbnavitia::Request& request) {
    pt::ptime start = pt::microsec_clock::universal_time();
    const auto mode = request.direct_path().streetnetwork_params().origin_mode();

    mode_costing.update_costing(make_modecosting_args(request.direct_path()));
    auto costing = mode_costing.get_costing_for_mode(mode);

    std::vector<midgard::PointLL> locations = util::convert_locations_to_pointLL(std::vector<pbnavitia::LocationContext>{request.direct_path().origin(),
                                                                                                                         request.direct_path().destination()});

    // It's a direct path.. we don't pollute the cache with random coords...
    const bool use_cache = false;
    auto projected_locations = projector(begin(locations), end(locations), graph, mode, costing, use_cache);

    auto coord_to_str = [](const auto& point) -> std::string {
        return std::to_string(point.lon()) + ";" + std::to_string(point.lat());
    };

    if (projected_locations.size() != 2) {
        auto err_message = "Cannot project the given coords: " + coord_to_str(request.direct_path().origin()) + ", " + coord_to_str(request.direct_path().destination());
        return make_error_response(pbnavitia::Error::no_origin_nor_destination, err_message);
    }

    valhalla::Location origin;
    valhalla::Location dest;
    baldr::PathLocation::toPBF(projected_locations.at(locations.front()), &origin, graph);
    baldr::PathLocation::toPBF(projected_locations.at(locations.back()), &dest, graph);

    auto& algo = get_path_algorithm(origin, dest, mode);

    const auto path_info_list = algo.GetBestPath(origin,
                                                 dest,
                                                 graph,
                                                 mode_costing.get_costing(),
                                                 util::convert_navitia_to_valhalla_mode(mode));

    // If no solution was found
    if (path_info_list.empty()) {
        pbnavitia::Response response;
        response.set_response_type(pbnavitia::NO_SOLUTION);
        LOG_ERROR("No solution found !");
        return response;
    }

    // The path algorithms all are allowed to return more than one path now.
    // None of them do, but the api allows for it
    // We just take the first and only one then
    valhalla::Options options;
    const auto& pathedges = path_info_list.front();
    thor::AttributesController controller;
    valhalla::Api api;
    auto* trip_leg = api.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
    thor::TripLegBuilder::Build(options, controller, graph, mode_costing.get_costing(), pathedges.begin(),
                                pathedges.end(), origin, dest, {}, *trip_leg, {"route"}, nullptr, nullptr);

    api.mutable_options()->set_language(request.direct_path().streetnetwork_params().language());
    odin::DirectionsBuilder::Build(api);

    const auto response = direct_path_response_builder::build_journey_response(request, pathedges, *trip_leg, api);

    auto duration = pt::microsec_clock::universal_time() - start;
    metrics.observe_handle_direct_path(mode, duration.total_milliseconds() / 1000.0);
    return response;
}

} // namespace asgard
