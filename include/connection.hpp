//
// Created by Raghav Anand on 3/28/20.
//

#ifndef MPLAMBDA_CONNECTION_HPP
#define MPLAMBDA_CONNECTION_HPP

#include <coordinator.hpp>
#include <jilog.hpp>
#include <getopt.h>
#include <string>
#include <demo/app_options.hpp>
#include <demo/png_2d_scenario.hpp>
#include <comm.hpp>
#include <poll.h>
#include <packet.hpp>
#include <buffer.hpp>
#include <write_queue.hpp>

namespace mpl {
    template <class Coordinator>
    class Connection {
        using ID = std::uint64_t;

        Coordinator& coordinator_;
        struct sockaddr_in addr_;
        socklen_t addrLen_{sizeof(addr_)};

        int socket_{-1};

        Buffer rBuf_{1024*4};
        WriteQueue writeQueue_;
        ID lambdaId_{0};
        bool recvHello_{false};

        // Delayed vertex write variables
        std::vector<std::vector<typename Coordinator::Vertex>> vertices_to_send;
        std::chrono::high_resolution_clock::time_point last_send_of_vertices;
        constexpr static double milliseconds_before_next_send = 1000; // TODO: make this smarter

//        ID groupId_{0};

        bool doRead() {
            assert(rBuf_.remaining() > 0); // we may need to grow the buffer

            ssize_t n = ::recv(socket_, rBuf_.begin(), rBuf_.remaining(), 0);
            JI_LOG(TRACE) << "recv " << n;
            if (n <= 0) {
                // on error (-1) or connection close (0), send DONE to
                // the group to which this connection is attached.
//                if (groupId_) {
//                    coordinator_.done(groupId_, this);
//                    groupId_ = 0;
//                }

                return (n < 0) ? throw syserr("recv") : false;
            }

            rBuf_ += n;
            rBuf_.flip();
            // call the appropriate process overload for each packet
            // that arrives
            std::size_t needed;
            while ((needed = packet::parse<
                    typename Coordinator::Edge,
                    typename Coordinator::Distance,
                    typename Coordinator::Vertex,
                    typename Coordinator::State>(rBuf_, [&] (auto&& pkt) {
                      process(std::forward<decltype(pkt)>(pkt));
            })) == 0);
            rBuf_.compact(needed);
            return true;
        }

        void process(packet::Hello&& pkt) {
            JI_LOG(INFO) << "got HELLO (id=" << pkt.id() << ")";
            recvHello_ = true;
            lambdaId_ = pkt.id();
            last_send_of_vertices = std::chrono::high_resolution_clock::now();

//            groupId_ = coordinator_.addToGroup(pkt.id(), this);
            // this is a possible sign that the group already ended
            // before this connection arrived.  Respond with DONE.
//            if (groupId_ == 0)
//                writeQueue_.push_back(packet::Done(pkt.id()));
        }

        void process(packet::Done&& pkt) {
            JI_LOG(INFO) << "got DONE (id=" << pkt.id() << ")";
//            if (groupId_ == 0 || groupId_ != pkt.id()) {
//                JI_LOG(WARN) << "DONE group id mismatch";
//            } else {
//                coordinator_.done(groupId_, this);
//                groupId_ = 0;
//            }
        }

        void process(packet::NumSamples&& pkt) {
            JI_LOG(ERROR) << "got NUM_SAMPLES"; // Coordinator should never receive this
//            if (groupId_ == 0 || groupId_ != pkt.id()) {
//                JI_LOG(WARN) << "DONE group id mismatch";
//            } else {
//                coordinator_.done(groupId_, this);
//                groupId_ = 0;
//            }
        }

        template <class Vertex, class State>
        void process(packet::Vertices<Vertex, State>&& pkt) {
            JI_LOG(INFO) << "got VERTICES";
            if (coordinator_.algorithm() == "prm_fixed_graph") {
                if (pkt.destination() == 0) {
                    coordinator_.update_num_samples(lambdaId_, pkt.vertices().size());
                    coordinator_.addVertices(std::move(pkt.vertices()));
                } else {
                    auto destinationLambdaId = pkt.destinationLambdaId();
                    auto other_connection = coordinator_.getConnection(destinationLambdaId);
                    if (other_connection != nullptr) {
                        // TODO: do a buffered write, the lambdas are receiving too many packets. Merge vertices from many outputs together.
                        other_connection->delayed_vertices_write(std::move(pkt));
                    } else {
                        coordinator_.buffered_data_[destinationLambdaId].push_back(std::move(pkt));
                    }
                }
            }
        }

        template <class Vertex, class State>
        void delayed_vertices_write(packet::Vertices<Vertex, State>&& pkt) {
            vertices_to_send.push_back(std::move(pkt.vertices()));
            perform_delayed_vertices_write();
        }

        void perform_delayed_vertices_write() {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_of_vertices);
            if ((vertices_to_send.size() >= coordinator_.smallest_number_of_neighbors) ||
                    (vertices_to_send.size() > 0 && duration.count() > milliseconds_before_next_send)) { // TODO: unverified strategy
                std::size_t total_size = 0;
                for (const auto& sub : vertices_to_send)
                    total_size += sub.size();
                std::vector<typename Coordinator::Vertex> result;
                result.reserve(total_size);
                for (const auto& sub : vertices_to_send)
                    result.insert(result.end(), sub.begin(), sub.end());
                JI_LOG(INFO) << "Writing " << result.size() << " vertices to " << lambdaId_;
                auto pkt = packet::Vertices<
                        typename Coordinator::Vertex,
                        typename Coordinator::State>(0, 0, std::move(result));
                write(std::move(pkt));
                vertices_to_send.clear();
                last_send_of_vertices = std::chrono::high_resolution_clock::now();
            }
        }

        template <class Edge, class Distance>
        void process(packet::Edges<Edge, Distance>&& pkt) {
            JI_LOG(INFO) << "got EDGES";
            if (coordinator_.algorithm() == "prm_fixed_graph") {
                coordinator_.addEdges(std::move(pkt.edges()));
            }
        }

//        void process(packet::Problem&& pkt) {
//            JI_LOG(INFO) << "got Problem from " << socket_;
//
            // if this connection is connected to a group, send DONE
            // to that group before starting a new group.
//            if (groupId_) {
//                coordinator_.done(groupId_, this);
//                groupId_ = 0;
//            }
//
//            groupId_ = coordinator_.createGroup(this, pkt.algorithm());
//            coordinator_.launchLambdas(groupId_, std::move(pkt));
//        }

//        template <class State>
//        void process(packet::Path<State>&& pkt) {
//            JI_LOG(INFO) << "got Path " << sizeof(State);
//            for (auto& q : pkt.path())
//                JI_LOG(TRACE) << "  " << q;
//
//            if (groupId_ == 0) {
//                JI_LOG(WARN) << "got PATH without active group";
//            } else {
//                coordinator_.gotPath(groupId_, std::move(pkt), this);
//            }
            // if (group_->second.algorithm() == 'r') {
            //     // for RRT we only send the path to the initiator
            //     group_->second.initiator()->write(std::move(pkt));
            // } else {
            //     // for C-FOREST we send broadcast to path
            //     coordinator_.broadcast(std::move(pkt), group_, this);
            // }
//        }

    public:
        explicit Connection(Coordinator& coordinator)
                : coordinator_(coordinator)
                , socket_(coordinator.accept(reinterpret_cast<struct sockaddr*>(&addr_), &addrLen_))
        {
            JI_LOG(TRACE) << "connection accepted";
        }

        ~Connection() {
//            if (groupId_) {
//                coordinator_.done(groupId_, this);
//                groupId_ = 0;
//            }

            JI_LOG(TRACE) << "closing connection";
            if (socket_ != -1 && ::close(socket_) == -1)
                JI_LOG(WARN) << "connection close error: " << errno;
        }

        operator bool () const {
            return socket_ != -1;
        }

        operator struct pollfd () const {
            return { socket_, static_cast<short>(writeQueue_.empty() ? POLLIN : (POLLIN | POLLOUT)), 0 };
        }

        bool recvHello() {
            return recvHello_;
        }

        ID lambdaId() {
            return lambdaId_;
        }


        void sendDone() {
            writeQueue_.push_back(packet::Done(0));
        }

        void degroup() {
//            groupId_ = 0;
        }

        template <class Packet>
        void write(Packet&& packet) {
            writeQueue_.push_back(std::forward<Packet>(packet));
        }

        bool process(const struct pollfd& pfd) {
            try {
                if ((pfd.revents & POLLIN) && !doRead())
                    return false;

                if (pfd.revents & POLLOUT)
                    writeQueue_.writeTo(socket_);

                return true;
            } catch (const std::exception& ex) {
                JI_LOG(WARN) << "exception processing connection: " << ex.what();
                return false;
            }
        }
    };


};
#endif //MPLAMBDA_CONNECTION_HPP
