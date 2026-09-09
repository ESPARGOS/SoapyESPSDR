#pragma once

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

// The C5 has fixed CDC-ACM endpoints. Never use the S31 vendor requests here.
// One caller owns a complete command/response transaction at a time.
class C5Serial {
public:
    using Clock = std::chrono::steady_clock;
    using Deadline = Clock::time_point;
    explicit C5Serial(const std::string &path) {
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) throw std::runtime_error("C5 serial open: " + std::string(strerror(errno)));
        try {
            if (flock(fd, LOCK_EX | LOCK_NB) < 0) throw std::runtime_error("C5 serial port busy");
            termios cfg{};
            if (tcgetattr(fd, &cfg) < 0) throw std::runtime_error("C5 serial tcgetattr failed");
            cfmakeraw(&cfg);
            cfg.c_cflag |= CLOCAL | CREAD;
            cfg.c_cflag &= ~(HUPCL | CRTSCTS);
            cfsetispeed(&cfg, B115200);
            cfsetospeed(&cfg, B115200);
            if (tcsetattr(fd, TCSANOW, &cfg) < 0) throw std::runtime_error("C5 serial tcsetattr failed");
            tcflush(fd, TCIFLUSH);
        } catch (...) { ::close(fd); fd=-1; throw; }
    }
    ~C5Serial() { if (fd >= 0) ::close(fd); }
    C5Serial(const C5Serial &) = delete;
    C5Serial &operator=(const C5Serial &) = delete;

    void write(const void *data, size_t bytes, Deadline deadline) {
        auto p=static_cast<const uint8_t *>(data);
        while (bytes) {
            wait(POLLOUT, deadline);
            const ssize_t n=::write(fd,p,bytes);
            if (n<0 && (errno==EINTR || errno==EAGAIN)) continue;
            if (n<=0) throw std::runtime_error("C5 serial write failed");
            p+=n;bytes-=n;
        }
    }
    void read(void *data, size_t bytes, Deadline deadline) {
        auto p=static_cast<uint8_t *>(data);
        while (bytes) {
            wait(POLLIN, deadline);
            const ssize_t n=::read(fd,p,bytes);
            if (n<0 && (errno==EINTR || errno==EAGAIN)) continue;
            if (n<=0) throw std::runtime_error("C5 serial disconnected");
            p+=n;bytes-=n;
        }
    }
    void command(const std::string &text, Deadline deadline) {
        const auto wire=text+"\n";
        write(wire.data(),wire.size(),deadline);
    }
    std::string line(Deadline deadline) {
        std::string text;
        for (;;) {
            char ch;read(&ch,1,deadline);
            if (ch=='\n') return text;
            if (ch!='\r') text+=ch;
            if (text.size()>255) throw std::runtime_error("C5 serial response line too long");
        }
    }
private:
    int fd=-1;
    void wait(short events, Deadline deadline) {
        for (;;) {
            auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
            if (remaining<=0) throw std::runtime_error("C5 serial timeout");
            pollfd p{fd,events,0};
            int result=::poll(&p,1,static_cast<int>(std::min<int64_t>(remaining,1000)));
            if (result<0 && errno==EINTR) continue;
            if (result<0 || (p.revents & (POLLERR|POLLNVAL))) throw std::runtime_error("C5 serial I/O error");
            if (p.revents & events) return;
            if (p.revents & POLLHUP) throw std::runtime_error("C5 serial disconnected");
        }
    }
};
