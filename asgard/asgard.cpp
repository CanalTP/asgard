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

#include "context.h"
#include "handler.h"
#include "utils/exception.h"
#include "utils/zmq.h"

#include "asgard/asgard_conf.h"
#include "asgard/metrics.h"
#include "asgard/projector.h"
#include "asgard/request.pb.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

using namespace valhalla;

static void respond(zmq::socket_t& socket,
                    const std::string& address,
                    const pbnavitia::Response& response) {
    zmq::message_t reply(response.ByteSize());
    try {
        response.SerializeToArray(reply.data(), response.ByteSize());
    } catch (const google::protobuf::FatalException& e) {
        pbnavitia::Response error_response;
        error_response.mutable_error()->set_id(pbnavitia::Error::internal_error);
        error_response.mutable_error()->set_message(e.what());
        reply.rebuild(error_response.ByteSize());
        error_response.SerializeToArray(reply.data(), error_response.ByteSize());
    }
    z_send(socket, address, ZMQ_SNDMORE);
    z_send(socket, "", ZMQ_SNDMORE);
    socket.send(reply);
}

static void worker(const asgard::Context& context) {
    zmq::context_t& zmq_context = context.zmq_context;
    asgard::Handler handler(context);

    zmq::socket_t socket(zmq_context, ZMQ_REQ);
    socket.connect("inproc://workers");
    z_send(socket, "READY");

    while (true) {

        const std::string address = z_recv(socket);
        {
            std::string empty = z_recv(socket);
            assert(empty.size() == 0);
        }

        zmq::message_t request;
        socket.recv(&request);
        asgard::InFlightGuard in_flight_guard(context.metrics.start_in_flight());
        pbnavitia::Request pb_req;
        if (!pb_req.ParseFromArray(request.data(), request.size())) {
            LOG_ERROR("receive invalid protobuf");
            pbnavitia::Response response;
            auto* error = response.mutable_error();
            error->set_id(pbnavitia::Error::invalid_protobuf_request);
            error->set_message("receive invalid protobuf");
            respond(socket, address, response);
            continue;
        }

        const auto response = handler.handle(pb_req);

        respond(socket, address, response);
    }
}

int main() {
    asgard::AsgardConf asgard_conf{};

    boost::thread_group threads;
    zmq::context_t context(1);
    LoadBalancer lb(context);
    lb.bind(asgard_conf.socket_path, "inproc://workers");
    const asgard::Metrics metrics(asgard_conf);
    const asgard::Projector projector(asgard_conf.cache_size["walking"],
                                      asgard_conf.cache_size["bike"],
                                      asgard_conf.cache_size["car"],
                                      asgard_conf.reachability,
                                      asgard_conf.radius);
    valhalla::baldr::GraphReader graph(asgard_conf.valhalla_conf.get_child("mjolnir"));

    for (size_t i = 0; i < asgard_conf.nb_threads; ++i) {
        threads.create_thread(std::bind(&worker, asgard::Context(context,
                                                                 graph,
                                                                 metrics,
                                                                 projector)));
    }

    // Connect worker threads to client threads via a queue
    while (true) {
        try {
            lb.run();
        } catch (const navitia::recoverable_exception& e) {
            LOG_ERROR(e.what());
        } catch (const zmq::error_t&) {} //lors d'un SIGHUP on restore la queue
    }

    return 0;
}
