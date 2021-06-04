#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE projector_test

#include "tile_maker.h"

#include "asgard/mode_costing.h"
#include "asgard/projector.h"
#include "asgard/util.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/test/unit_test.hpp>

#include <valhalla/midgard/pointll.h>

using namespace valhalla;

namespace asgard {

class UnitTestProjector {
public:
    UnitTestProjector(size_t cache_size_walking = 5,
                      size_t cache_size_bike = 5,
                      size_t cache_size_car = 5,
                      unsigned int min_outbound_reach = 0,
                      unsigned int min_inbound_reach = 0,
                      unsigned int radius = 0) : p(cache_size_walking,
                                                   cache_size_bike,
                                                   cache_size_car,
                                                   min_outbound_reach, min_inbound_reach, radius) {}

    valhalla::baldr::Location build_location(const std::string& place,
                                             unsigned int min_outbound_reach,
                                             unsigned int min_inbound_reach,
                                             unsigned int radius) const {
        auto c = navitia::parse_coordinate(place);
        auto pointll = valhalla::midgard::PointLL{c.first, c.second};
        return p.build_location(pointll, min_outbound_reach, min_inbound_reach, radius);
    }

private:
    Projector p;
};

std::vector<valhalla::midgard::PointLL>
make_pointLLs(const std::vector<std::string> coords) {
    std::vector<valhalla::midgard::PointLL> pointLLs;
    std::transform(coords.begin(), coords.end(), std::back_inserter(pointLLs), [](const auto& coord) {
        auto c = navitia::parse_coordinate(coord);
        return valhalla::midgard::PointLL{c.first, c.second};
    });
    return pointLLs;
}

BOOST_AUTO_TEST_CASE(simple_projector_test) {
    tile_maker::TileMaker maker;
    maker.make_tile();

    boost::property_tree::ptree conf;
    conf.put("tile_dir", maker.get_tile_dir());
    valhalla::baldr::GraphReader graph(conf);

    ModeCosting mode_costing;
    auto costing = mode_costing.get_costing_for_mode("car");
    Projector p(2, 2, 2);
    // cache = {}
    {
        auto locations = make_pointLLs({"coord:2:2"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 0);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 1);
    }
    // cache = {}
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 2);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 2);
    }
    // cache = { coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 2);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 3);
    }
    // cache = { coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.009:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 3);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 4);
    }
    // cache = { coord:.009:.001; coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 3);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 5);
    }
    // cache = { coord:.003:.001; coord:.009:.001 }
    {
        auto locations = make_pointLLs({"coord:.013:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 4);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 6);
    }
    // cache = { coord:.013:.001; coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.009:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 5);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 7);
    }
    // cache = { coord:.009:.001; coord:.013:.001 }
}

BOOST_AUTO_TEST_CASE(multi_cache_test) {
    tile_maker::TileMaker maker;
    maker.make_tile();

    boost::property_tree::ptree conf;
    conf.put("tile_dir", maker.get_tile_dir());
    valhalla::baldr::GraphReader graph(conf);

    ModeCosting mode_costing;
    auto costing_car = mode_costing.get_costing_for_mode("car");
    auto costing_bike = mode_costing.get_costing_for_mode("bike");
    auto costing_walking = mode_costing.get_costing_for_mode("walking");
    Projector p(2, 2, 2);
    // Use car cache other cache should not be affected !!!
    // car : cache = {}
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing_car);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 1);
    }
    // car : cache = { coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing_car);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 2);
    }
    // car : cache = { coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.009:.001"});
        auto result = p(begin(locations), end(locations), graph, "car", costing_car);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 2);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 3);
    }
    // Now use walking cache other cache should not be affected !!!
    // walking : cache = {}
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "walking", costing_walking);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("walking"), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("walking"), 1);
    }
    // walking : cache = { coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.003:.001"});
        auto result = p(begin(locations), end(locations), graph, "walking", costing_walking);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("walking"), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("walking"), 2);
    }
    // walking : cache = { coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.009:.001"});
        auto result = p(begin(locations), end(locations), graph, "walking", costing_walking);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("walking"), 2);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("walking"), 3);
    }
    // walking : cache = { coord:.009:.001; coord:.003:.001 }
    {
        auto locations = make_pointLLs({"coord:.004:.005"});
        auto result = p(begin(locations), end(locations), graph, "walking", costing_walking);
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(p.get_nb_cache_miss("walking"), 3);
        BOOST_CHECK_EQUAL(p.get_nb_cache_calls("walking"), 4);
    }

    BOOST_CHECK_EQUAL(p.get_nb_cache_miss("car"), 2);
    BOOST_CHECK_EQUAL(p.get_nb_cache_calls("car"), 3);
    BOOST_CHECK_EQUAL(p.get_nb_cache_miss("bike"), 0);
    BOOST_CHECK_EQUAL(p.get_nb_cache_calls("bike"), 0);
    // Test not created cache (only walking, car and bike exists)
    BOOST_CHECK_EQUAL(p.get_nb_cache_miss("none"), 0);
    BOOST_CHECK_EQUAL(p.get_nb_cache_calls("none"), 0);
}

BOOST_AUTO_TEST_CASE(build_location_test) {
    UnitTestProjector testProjector(3, 3, 3);
    {
        BOOST_CHECK_THROW(testProjector.build_location("plop", 0, 0, 0), navitia::wrong_coordinate);
    }
    {
        const auto l = testProjector.build_location("coord:8:0", 12u, 12u, 42l);
        BOOST_CHECK_CLOSE(l.latlng_.lng(), 8.f, .0001f);
        BOOST_CHECK_CLOSE(l.latlng_.lat(), 0.f, .0001f);
        BOOST_CHECK_EQUAL(static_cast<bool>(l.stoptype_), false);
        BOOST_CHECK_EQUAL(l.radius_, 42l);
    }
    {
        const auto l = testProjector.build_location("coord:8:0", 12u, 12u, 42l);
        BOOST_CHECK_CLOSE(l.latlng_.lng(), 8.f, .0001f);
        BOOST_CHECK_CLOSE(l.latlng_.lat(), 0.f, .0001f);
        BOOST_CHECK_EQUAL(static_cast<bool>(l.stoptype_), false);
        BOOST_CHECK_EQUAL(l.radius_, 42l);
        BOOST_CHECK_EQUAL(l.latlng_.lng(), 8);
        BOOST_CHECK_EQUAL(l.latlng_.lat(), 0);
    }
    {
        const auto l = testProjector.build_location("92;43", 29u, 29u, 15l);
        BOOST_CHECK_CLOSE(l.latlng_.lng(), 92.f, .0001f);
        BOOST_CHECK_CLOSE(l.latlng_.lat(), 43.f, .0001f);
        BOOST_CHECK_EQUAL(static_cast<bool>(l.stoptype_), false);
        BOOST_CHECK_EQUAL(l.radius_, 15l);
        BOOST_CHECK_EQUAL(l.latlng_.lng(), 92);
        BOOST_CHECK_EQUAL(l.latlng_.lat(), 43);
    }
}

} // namespace asgard
