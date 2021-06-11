#pragma once

#include <prometheus/exposer.h>
#include <boost/core/noncopyable.hpp>
#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <string>

namespace prometheus {
class Registry;
class Histogram;
class Gauge;
} // namespace prometheus

namespace asgard {

struct AsgardConf;

class InFlightGuard {
    prometheus::Gauge* gauge;

public:
    explicit InFlightGuard(prometheus::Gauge* gauge);
    InFlightGuard(InFlightGuard& other) = delete;
    InFlightGuard(InFlightGuard&& other) noexcept;
    void operator=(InFlightGuard& other) = delete;
    InFlightGuard& operator=(InFlightGuard&& other) noexcept;
    ~InFlightGuard();
};

class Metrics : boost::noncopyable {
protected:
    std::unique_ptr<prometheus::Exposer> exposer;
    std::shared_ptr<prometheus::Registry> registry;
    prometheus::Gauge* in_flight;
    prometheus::Gauge* status_family;
    std::map<const std::string, prometheus::Histogram*> handle_direct_path_histogram;
    std::map<const std::string, prometheus::Histogram*> handle_matrix_histogram;
    std::unordered_map<std::string, prometheus::Gauge*> nb_cache_miss_gauge;
    std::unordered_map<std::string, prometheus::Gauge*> nb_cache_call_gauge;
    std::unordered_map<std::string, prometheus::Gauge*> current_cache_size;

public:
    explicit Metrics(const boost::optional<const AsgardConf&>& config);
    InFlightGuard start_in_flight() const;

    void observe_handle_direct_path(const std::string&, double duration) const;
    void observe_handle_matrix(const std::string&, double duration) const;
    void observe_nb_cache_miss(const std::string& mode, uint64_t nb_cache_miss, uint64_t nb_cache_calls) const;
    void observe_cache_size(const std::string& mode, uint64_t cache_size) const;
};

} // namespace asgard
