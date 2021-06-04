#pragma once

#include <valhalla/midgard/logging.h>

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace {

template<typename T>
const T get_config(const std::string& key, T value = T()) {
    char* v = std::getenv(key.c_str());
    if (v != nullptr) {
        value = boost::lexical_cast<T>(v);
    }
    LOG_INFO("Config: " + key + "=" + boost::lexical_cast<std::string>(value));
    return value;
}

void configure_logs(const std::string& key) {
    const auto* v = std::getenv(key.c_str());
    if (v == nullptr) {
        LOG_INFO("Using default log configuration. Logs are going to std_out");
        return;
    }

    const auto logging_file_path = boost::lexical_cast<std::string>(v);
    valhalla::midgard::logging::Configure({{"type", "file"}, {"file_name", logging_file_path}, {"reopen_interval", "1"}});
}

} // namespace

namespace asgard {

namespace ptree = boost::property_tree;
struct AsgardConf {
    std::string socket_path;
    std::unordered_map<std::string, std::size_t> cache_size;
    std::size_t nb_threads;
    ptree::ptree valhalla_conf;
    boost::optional<std::string> metrics_binding;
    unsigned int reachability;
    unsigned int radius;

    AsgardConf() {
        configure_logs("ASGARD_LOGGING_FILE_PATH");
        socket_path = get_config<std::string>("ASGARD_SOCKET_PATH", "tcp://*:6000");
        cache_size["walking"] = get_config<size_t>("ASGARD_WALKING_CACHE_SIZE", 1000000);
        cache_size["bike"] = get_config<size_t>("ASGARD_BIKE_CACHE_SIZE", 1000000);
        cache_size["car"] = get_config<size_t>("ASGARD_CAR_CACHE_SIZE", 1000000);
        nb_threads = get_config<size_t>("ASGARD_NB_THREADS", 3);
        metrics_binding = get_config<std::string>("ASGARD_METRICS_BINDING", std::string("0.0.0.0:8080"));

        auto valhalla_conf_json = get_config<std::string>("ASGARD_VALHALLA_CONF", "/data/valhalla/valhalla.json");
        ptree::read_json(valhalla_conf_json, valhalla_conf);

        reachability = valhalla_conf.get<unsigned int>("loki.service_defaults.minimum_reachability", 0);
        radius = valhalla_conf.get<unsigned int>("loki.service_defaults.radius", 0);
    }
};

} // namespace asgard
