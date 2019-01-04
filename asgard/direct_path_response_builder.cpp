#include "direct_path_response_builder.h"

#include "util.h"

#include "asgard/request.pb.h"

#include <valhalla/midgard/logging.h>
#include <valhalla/thor/pathinfo.h>

#include <numeric>

using namespace valhalla;

namespace asgard {

namespace direct_path_response_builder {

pbnavitia::Response build_journey_response(const pbnavitia::Request& request,
                                           const std::vector<thor::PathInfo>& path_info_list,
                                           float total_length) {
    pbnavitia::Response response;

    if (path_info_list.empty()) {
        response.set_response_type(pbnavitia::NO_SOLUTION);
        LOG_ERROR("No solution found !");
        return response;
    }

    LOG_INFO("Building solution...");
    // General
    response.set_response_type(pbnavitia::ITINERARY_FOUND);

    // Journey
    auto* journey = response.mutable_journeys()->Add();
    journey->set_duration(path_info_list.back().elapsed_time);
    journey->set_nb_transfers(0);
    journey->set_nb_sections(1);

    auto const departure_posix_time = time_t(request.direct_path().datetime());
    auto const arrival_posix_time = departure_posix_time + time_t(journey->duration());

    journey->set_requested_date_time(departure_posix_time);
    journey->set_departure_date_time(departure_posix_time);
    journey->set_arrival_date_time(arrival_posix_time);

    // Section
    auto* s = journey->add_sections();
    s->set_type(pbnavitia::STREET_NETWORK);
    s->set_id("section");
    s->set_duration(journey->duration());
    // We take the mode of the first path. Could be the last too...
    // They could also be different in the list...
    s->mutable_street_network()->set_mode(asgard::util::convert_valhalla_to_navitia_mode(path_info_list.front().mode));
    s->set_begin_date_time(departure_posix_time);
    s->set_end_date_time(arrival_posix_time);

    s->set_length(total_length);

    compute_metadata(*journey);

    LOG_INFO("Solution built...");

    return response;
}

void compute_metadata(pbnavitia::Journey& pb_journey) {
    uint32_t total_walking_duration = 0;
    uint32_t total_car_duration = 0;
    uint32_t total_bike_duration = 0;
    uint32_t total_ridesharing_duration = 0;
    uint32_t total_walking_distance = 0;
    uint32_t total_car_distance = 0;
    uint32_t total_bike_distance = 0;
    uint32_t total_ridesharing_distance = 0;

    for (const auto& section: pb_journey.sections()) {
        if (section.type() == pbnavitia::STREET_NETWORK || section.type() == pbnavitia::CROW_FLY) {
            switch (section.street_network().mode()) {
            case pbnavitia::StreetNetworkMode::Walking:
                total_walking_duration += section.duration();
                total_walking_distance += section.length();
                break;
            case pbnavitia::StreetNetworkMode::Car:
            case pbnavitia::StreetNetworkMode::CarNoPark:
                total_car_duration += section.duration();
                total_car_distance += section.length();
                break;
            case pbnavitia::StreetNetworkMode::Bike:
            case pbnavitia::StreetNetworkMode::Bss:
                total_bike_duration += section.duration();
                total_bike_distance += section.length();
                break;
            case pbnavitia::StreetNetworkMode::Ridesharing:
                total_ridesharing_duration += section.duration();
                total_ridesharing_distance += section.length();
                break;
            }
        } else if (section.type() == pbnavitia::TRANSFER && section.transfer_type() == pbnavitia::walking) {
            total_walking_duration += section.duration();
        }
    }

    const auto ts_departure = pb_journey.sections(0).begin_date_time();
    const auto ts_arrival = pb_journey.sections(pb_journey.sections_size() - 1).end_date_time();
    auto* durations = pb_journey.mutable_durations();
    durations->set_walking(total_walking_duration);
    durations->set_bike(total_bike_duration);
    durations->set_car(total_car_duration);
    durations->set_ridesharing(total_ridesharing_duration);
    durations->set_total(ts_arrival - ts_departure);

    int i = 0;
    i++;

    auto* distances = pb_journey.mutable_distances();
    distances->set_walking(total_walking_distance);
    distances->set_bike(total_bike_distance);
    distances->set_car(total_car_distance);
    distances->set_ridesharing(total_ridesharing_distance);
}
}

} // namespace asgard::direct_path_response_builder
