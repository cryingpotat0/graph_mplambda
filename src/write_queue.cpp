#include <jilog.hpp>
#include <write_queue.hpp>
#include <syserr.hpp>
#include <sys/socket.h>


void mpl::WriteQueue::writeTo(int socket) {
    if (empty())
        return;

    iovs_.clear();
    for (auto it = buffers_.begin() ; iovs_.size() < MAX_IOVS && it != buffers_.end() ; ++it) {
        iovs_.emplace_back();
        iovs_.back().iov_base = it->begin();
        iovs_.back().iov_len = it->remaining();
    }

    //JI_LOG(TRACE) << "about to write " << iovs_.size() << " iovecs to " << socket;
    ssize_t n = ::writev(socket, iovs_.data(), iovs_.size());
    //struct msghdr mh;
    //mh.msg_iov = iovs_.data();
    //mh.msg_iovlen = iovs_.size();
    //struct sockaddr_in dest;
    //ssize_t n = ::sendmsg(socket, &mh, MSG_NOSIGNAL);

    //JI_LOG(TRACE) << "wrote " << n << " bytes to " << socket;
    if (n == -1) {
        if (errno == EAGAIN)
            return;
	JI_LOG(TRACE) << errno;
        throw syserr("writev");
    }

    while (n > 0) {
        if (n >= buffers_.front().remaining()) {
            n -= buffers_.front().remaining();
            buffers_.pop_front();
//            JI_LOG(TRACE) << "removing completed buffer";
        } else {
//            JI_LOG(TRACE) << "updating buffer in front";
            buffers_.front() += n;
            break;
        }
    }
}

