//
// Created by Raghav Anand on 3/28/20.
//

#ifndef MPLAMBDA_CONNECTION_HPP
#define MPLAMBDA_CONNECTION_HPP

#include <buffer.hpp>
#include <comm.hpp>
#include <coordinator.hpp>
#include <demo/app_options.hpp>
#include <demo/png_2d_scenario.hpp>
#include <getopt.h>
#include <jilog.hpp>
#include <packet.hpp>
#include <poll.h>
#include <string>
#include <write_queue.hpp>

namespace mpl {
    template <class Coordinator> class Connection {
        using ID = std::uint64_t;

        Coordinator &coordinator_;
        struct sockaddr_in addr_;
        socklen_t addrLen_{sizeof(addr_)};

        int socket_{-1};

        Buffer rBuf_{1024 * 4};
        WriteQueue writeQueue_;
        ID lambdaId_{0};
        bool recvHello_{false};
        bool recvDone_{false};
        std::uint64_t start_time_millis_{0};
        int curr_id_loc{0};

        // Delayed vertex write variables
        std::vector<std::vector<typename Coordinator::Vertex>> vertices_to_send;

        //        ID groupId_{0};

        bool doRead() {
            assert(rBuf_.remaining() > 0); // we may need to grow the buffer

            ssize_t n = ::recv(socket_, rBuf_.begin(), rBuf_.remaining(), 0);
            // JI_LOG(INFO) << "recv " << n;
            if (n <= 0) {

                return (n < 0) ? throw syserr("recv") : false;
            }

            rBuf_ += n;
            rBuf_.flip();
            // call the appropriate process overload for each packet
            // that arrives
            std::size_t needed;
            while ((needed = packet::parse<
                        typename Coordinator::Edge, typename Coordinator::Distance,
                        typename Coordinator::Vertex, typename Coordinator::State>(
                            rBuf_, [&](auto &&pkt) {
                            process(std::forward<decltype(pkt)>(pkt));
                            })) == 0)
            ;
            rBuf_.compact(needed);
            return true;
        }

        void process(packet::Hello &&pkt) {
            JI_LOG(INFO) << "got HELLO (id=" << pkt.id() << ")"
                << " socket " << socket_;
            recvHello_ = true;
            lambdaId_ = pkt.id();
            curr_id_loc = lambdaId_;
            start_time_millis_ = pkt.start_time();

//            groupId_ = coordinator_.addToGroup(pkt.id(), this);
            // this is a possible sign that the group already ended
            // before this connection arrived.  Respond with DONE.
//            if (groupId_ == 0)
//                writeQueue_.push_back(packet::Done(pkt.id()));
        }

        void process(packet::Done &&pkt) {
            JI_LOG(INFO) << "got DONE (id=" << pkt.id() << ")";
            recvDone_ = true;
        }

        void process(packet::NumSamples &&pkt) {
            JI_LOG(ERROR) << "got NUM_SAMPLES"; // Coordinator should never receive this
        }

        void process(packet::RandomSeedWork &&pkt) {
            JI_LOG(ERROR) << "got RANDOM_SEED_WORK"; // Coordinator should never receive this
        }

        template <class Vertex, class State>
            void process(packet::Vertices<Vertex, State> &&pkt) {
                /* if (curr_id_loc >= coordinator_.work_vec.size()) { */
                /*     sendDone(); */
                /*     return; */
                /* } */

                /* auto& curr_queue = coordinator_.work_vec[curr_id_loc]; */
                /* while (curr_queue.empty() && curr_id_loc < coordinator_.work_vec.size()) { */
                /*     curr_queue = coordinator_.work_vec[curr_id_loc]; */
                /*     curr_id_loc++; */
                /* } */

                /* if (curr_queue.empty()) { */
                /*     sendDone(); */
                /*     return; */
                /* } */

                /* auto& [start, end] = curr_queue.front(); */
                /* auto work_pkt = packet::RandomSeedWork(start, end); */
                /* curr_queue.pop(); */
                /* JI_LOG(INFO) << "Sending work pkt " << "(" << */
                /*     work_pkt.start_id() << "," << work_pkt.end_id() << */ 
                /*     ") to lambda " << lambdaId_; */
                /* coordinator_.writePacketToLambda(lambdaId_, lambdaId_, work_pkt); */
                
                if (coordinator_.work_queue.empty()) {
                    sendDone();
                    return;
                }
                auto work_pkt = coordinator_.work_queue.front();
                coordinator_.work_queue.pop();

                JI_LOG(INFO) << "Sending work pkt " << "(" <<
                    work_pkt.start_id() << "," << work_pkt.end_id() << 
                    ") to lambda " << lambdaId_;
                coordinator_.writePacketToLambda(lambdaId_, lambdaId_, work_pkt);
            }

        template <class Edge, class Distance>
            void process(packet::Edges<Edge, Distance> &&pkt) {
                // JI_LOG(INFO) << "Received " << pkt.edges().size() << " edges from lambda
                // " << lambdaId_;
                coordinator_.addEdges(std::move(pkt.edges()));
            }


        public:
        explicit Connection(Coordinator &coordinator)
            : coordinator_(coordinator),
            socket_(coordinator.accept(reinterpret_cast<struct sockaddr *>(&addr_),
                        &addrLen_)) {
                JI_LOG(TRACE) << "connection accepted";
            }

        ~Connection() {
            JI_LOG(TRACE) << "closing connection";
            if (socket_ != -1 && ::close(socket_) == -1)
                JI_LOG(WARN) << "connection close error: " << errno;
        }

        operator bool() const { return socket_ != -1; }

        operator struct pollfd() const {
            return {
                socket_,
                    static_cast<short>(writeQueue_.empty() ? POLLIN : (POLLIN | POLLOUT)),
                    0};
        }

        bool recvHello() { return recvHello_; }

        std::uint64_t helloStartTime() {
            return start_time_millis_;
        }

        bool recvDone() {
            return recvDone_;
        }

        ID lambdaId() { return lambdaId_; }

        void sendDone() { 
		writeQueue_.push_back(packet::Done(0)); 
		JI_LOG(INFO) << "sent done to lambda " << lambdaId_;
	}

        void degroup() { }

        template <class Packet> void write(Packet &&packet) {
            writeQueue_.push_back(std::forward<Packet>(packet));
        }

        void write_buf(Buffer &&buf) { writeQueue_.push_back(std::move(buf)); }

        bool process(const struct pollfd &pfd) {
            try {
                if ((pfd.revents & POLLIN) && !doRead())
                    return false;

                if (pfd.revents & POLLOUT)
                    writeQueue_.writeTo(socket_);

                return true;
            } catch (const std::exception &ex) {
		recvDone_ = true;
                JI_LOG(WARN) << "exception processing connection: " << ex.what()
                    << " lambda id " << lambdaId_;
                return false;
            }
        }
    };

};     // namespace mpl
#endif // MPLAMBDA_CONNECTION_HPP
