#include "metrics.h"

#include "asgard/asgard_conf.h"
#include "asgard/conf.h"

#include <prometheus/exposer.h>
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <prometheus/gauge_builder.h>
#include <prometheus/histogram.h>
#include <prometheus/histogram_builder.h>
#include <prometheus/registry.h>
#include <valhalla/midgard/logging.h>
#include <boost/none.hpp>
#include <boost/optional/detail/optional_relops.hpp>
#include <boost/optional/optional.hpp>

#include <iterator>
#include <utility>
#include <vector>

namespace asgard {

InFlightGuard::InFlightGuard(prometheus::Gauge* gauge) : gauge(gauge) {
    if (gauge != nullptr) {
        gauge->Increment();
    }
}

InFlightGuard::~InFlightGuard() {
    if (gauge != nullptr) {
        gauge->Decrement();
    }
}

InFlightGuard::InFlightGuard(InFlightGuard&& other) noexcept {
    this->gauge = other.gauge;
    other.gauge = nullptr;
}

InFlightGuard& InFlightGuard::operator=(InFlightGuard&& other) noexcept {
    this->gauge = other.gauge;
    other.gauge = nullptr;
    return *this;
}

static prometheus::Histogram::BucketBoundaries create_fixed_duration_buckets() {
    //boundaries need to be sorted!
    auto bucket_boundaries = prometheus::Histogram::BucketBoundaries{
        0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1, 1.5, 2, 5, 10};
    return bucket_boundaries;
}

Metrics::Metrics(const boost::optional<const AsgardConf&>& config) {
    if (config == boost::none) {
        return;
    }

    const auto& conf = config.get();
    if (conf.metrics_binding == boost::none) {
        return;
    }
    LOG_INFO("metrics available at http://" + *conf.metrics_binding + "/metrics");
    exposer = std::make_unique<prometheus::Exposer>(*conf.metrics_binding);
    registry = std::make_shared<prometheus::Registry>();
    exposer->RegisterCollectable(registry);

    const std::map<std::string, std::string> infos = {
        {"version", std::string(config::project_version)},
        {"build_type", std::string(config::asgard_build_type)},
        {"max_walking_cache_size", std::to_string(conf.cache_size.at("walking"))},
        {"max_bike_cache_size", std::to_string(conf.cache_size.at("bike"))},
        {"max_car_cache_size", std::to_string(conf.cache_size.at("car"))},
        {"nb_threads", std::to_string(conf.nb_threads)},
        {"reachability", std::to_string(conf.reachability)},
        {"radius", std::to_string(conf.radius)}};

    auto& status_family = prometheus::BuildGauge()
                              .Labels(infos)
                              .Name("asgard_status")
                              .Help("Status API of Asgard")
                              .Register(*registry);
    this->status_family = &status_family.Add({});

    auto& in_flight_family = prometheus::BuildGauge()
                                 .Name("asgard_request_in_flight")
                                 .Help("Number of requests currently being processed")
                                 .Register(*registry);
    this->in_flight = &in_flight_family.Add({});

    auto& direct_path_family = prometheus::BuildHistogram()
                                   .Name("asgard_direct_path_duration_seconds")
                                   .Help("duration of direct path computation")
                                   .Register(*registry);

    auto& matrix_family = prometheus::BuildHistogram()
                              .Name("asgard_handle_matrix_duration_seconds")
                              .Help("duration of matrix computation")
                              .Register(*registry);

    std::vector<std::string> list_modes = {"walking", "bike", "car", "taxi", "bss"};
    for (auto const& mode : list_modes) {
        auto& histo_direct_path = direct_path_family.Add({{"mode", mode}}, create_fixed_duration_buckets());
        this->handle_direct_path_histogram[mode] = &histo_direct_path;
        auto& histo_matrix = matrix_family.Add({{"mode", mode}}, create_fixed_duration_buckets());
        this->handle_matrix_histogram[mode] = &histo_matrix;
    }

    for(auto const& mode : {"walking", "bike", "car"}){
        nb_cache_miss_gauge[mode] = &prometheus::BuildGauge()
                                       .Name(std::string("nb_cache_miss_") + mode)
                                       .Help(std::string("Nb of projector's cache[") + mode + std::string("] miss from the start of app"))
                                       .Register(*registry)
                                       .Add({});

        nb_cache_call_gauge[mode] = &prometheus::BuildGauge()
                                       .Name(std::string("nb_cache_calls_") + mode)
                                       .Help(std::string("Nb of projector's cache[") + mode + std::string("] calls from the start of app"))
                                       .Register(*registry)
                                       .Add({});

        current_cache_size[mode] = &prometheus::BuildGauge()
                                      .Name(std::string("cache_size_") + mode)
                                      .Help(std::string("current cache[") + mode + std::string("]size"))
                                      .Register(*registry)
                                      .Add({});
    }
}

InFlightGuard Metrics::start_in_flight() const {
    if (!registry) {
        return InFlightGuard(nullptr);
    }
    return InFlightGuard(this->in_flight);
}

void Metrics::observe_handle_direct_path(const std::string& mode, double duration) const {
    if (!registry) {
        return;
    }
    auto it = this->handle_direct_path_histogram.find(mode);
    if (it != std::end(this->handle_direct_path_histogram)) {
        it->second->Observe(duration);
    } else {
        LOG_WARN("mode " + mode + " not found in metrics");
    }
}

void Metrics::observe_handle_matrix(const std::string& mode, double duration) const {
    if (!registry) {
        return;
    }
    auto it = this->handle_matrix_histogram.find(mode);
    if (it != std::end(this->handle_matrix_histogram)) {
        it->second->Observe(duration);
    } else {
        LOG_WARN("mode " + mode + " not found in metrics");
    }
}

void Metrics::observe_nb_cache_miss(const std::string& mode, uint64_t nb_cache_miss, uint64_t nb_cache_calls) const {
    if (!registry) {
        return;
    }
    nb_cache_miss_gauge.at(mode)->Set(nb_cache_miss);
    nb_cache_call_gauge.at(mode)->Set(nb_cache_calls);
}

void Metrics::observe_cache_size(const std::string& mode, uint64_t cache_size) const {
    if (!registry) {
        return;
    }
    current_cache_size.at(mode)->Set(cache_size);
}

} // namespace asgard
