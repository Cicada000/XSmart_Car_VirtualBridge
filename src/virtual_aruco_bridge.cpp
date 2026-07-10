#include "virtual_bridge/AppConfig.hpp"
#include "virtual_bridge/ControlFrame.hpp"
#include "virtual_bridge/RobotPositionJson.hpp"
#include "virtual_bridge/VehicleModel.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

std::atomic_bool g_running{true};

struct SharedCommand {
    std::mutex mutex;
    virtual_bridge::ControlCommand command;
    std::uint64_t frameCount = 0;
};

void handleSignal(int) {
    g_running.store(false);
}

void usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  --config PATH                Config file, default config/virtual_bridge.json\n"
        << "  --control-bind IP            Override TCP bind IP for XSmart control frames\n"
        << "  --control-port PORT          Override TCP control port\n"
        << "  --udp-host IP                Override UDP robot_position target host\n"
        << "  --udp-port PORT              Override UDP robot_position target port\n"
        << "  --send-hz HZ                 Override UDP publish frequency\n"
        << "  --initial-world-x-mm MM      Override initial localization point world X\n"
        << "  --initial-world-y-mm MM      Override initial localization point world Y\n"
        << "  --initial-heading-deg DEG    Override initial vehicle front direction\n"
        << "  --wheelbase-m M              Override front/rear axle distance\n"
        << "  --rear-track-m M             Override left/right rear wheel distance\n"
        << "  --pose-offset-m M            Override localization point offset from rear axle\n"
        << "  --servo-mid-us US            Override neutral servo pulse\n"
        << "  --servo-span-us US           Override pulse span for 180 degrees\n"
        << "  --max-steering-deg DEG       Override front wheel steering clamp\n"
        << "  --steering-sign SIGN         Override steering polarity, 1 or -1\n"
        << "  --servo-sec-per-60-deg S     Override servo speed model\n"
        << "  --speed-scale SCALE          Override command speed to simulated m/s multiplier\n"
        << "  --speed-tau-s S              Override first-order speed response time constant\n"
        << "  --max-accel-mps2 A           Override acceleration clamp\n"
        << "  --max-dt-s S                 Override integration dt clamp after pauses\n"
        << "  --clamp-negative-speed       Clamp reverse command speed to zero\n"
        << "  --allow-negative-speed       Allow reverse command speed\n"
        << "  --height-m M                 Override robot_position Y height\n"
        << "  --yaw-offset-deg DEG         Override robot_position yaw offset\n"
        << "  --yaw-sign SIGN              Override robot_position yaw sign\n"
        << "  --pos-x-source world_x|world_y  Override robot_position pos[0] source\n"
        << "  --pos-x-sign SIGN            Override robot_position pos[0] sign\n"
        << "  --pos-z-source world_x|world_y  Override robot_position pos[2] source\n"
        << "  --pos-z-sign SIGN            Override robot_position pos[2] sign\n"
        << "  --quiet                      Suppress periodic status output\n"
        << "  -h, --help                   Show help\n";
}

std::int64_t monotonicNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

int createTcpServer(const std::string& bindIp, int port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error(std::string("tcp socket failed: ") + std::strerror(errno));

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, bindIp.c_str(), &addr.sin_addr) <= 0) {
        ::close(fd);
        throw std::runtime_error("invalid control bind ip: " + bindIp);
    }
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        throw std::runtime_error(std::string("control bind failed: ") + std::strerror(errno));
    }
    if (::listen(fd, 4) < 0) {
        ::close(fd);
        throw std::runtime_error(std::string("control listen failed: ") + std::strerror(errno));
    }
    return fd;
}

int createUdpSocket(const std::string& host, int port, sockaddr_in& target) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error(std::string("udp socket failed: ") + std::strerror(errno));

    target = {};
    target.sin_family = AF_INET;
    target.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &target.sin_addr) <= 0) {
        ::close(fd);
        throw std::runtime_error("invalid udp host ip: " + host);
    }
    return fd;
}

void setLatestCommand(SharedCommand& shared, const virtual_bridge::ControlCommand& command) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.command = command;
    ++shared.frameCount;
}

std::pair<virtual_bridge::ControlCommand, std::uint64_t> latestCommand(SharedCommand& shared) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    return {shared.command, shared.frameCount};
}

void handleControlClient(int clientFd, SharedCommand& shared, bool quiet) {
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    setsockopt(clientFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::vector<std::uint8_t> pending;
    std::uint8_t buffer[256]{};
    while (g_running.load()) {
        const ssize_t n = ::recv(clientFd, buffer, sizeof(buffer), 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;
        }
        if (n == 0) break;

        std::vector<virtual_bridge::ControlCommand> commands;
        virtual_bridge::appendControlFrameBytes(
            pending, buffer, static_cast<std::size_t>(n), commands);
        for (const virtual_bridge::ControlCommand& command : commands) {
            setLatestCommand(shared, command);
            if (!quiet) {
                std::cout << "[control] speed=" << command.speedMps
                          << " servo=" << command.servoPulseUs << "\n";
            }
        }
    }
}

void controlServerLoop(const virtual_bridge::AppConfig& options, SharedCommand& shared) {
    try {
        const int serverFd = createTcpServer(options.controlBindIp, options.controlPort);
        if (!options.quiet) {
            std::cout << "virtual_aruco_bridge control_tcp="
                      << options.controlBindIp << ":" << options.controlPort << "\n";
        }

        while (g_running.load()) {
            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(serverFd, &readSet);
            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 200000;
            const int ready = ::select(serverFd + 1, &readSet, nullptr, nullptr, &timeout);
            if (ready < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (ready == 0) continue;

            sockaddr_in clientAddr{};
            socklen_t clientLen = sizeof(clientAddr);
            const int clientFd = ::accept(serverFd, reinterpret_cast<sockaddr*>(&clientAddr), &clientLen);
            if (clientFd < 0) {
                if (errno == EINTR) continue;
                continue;
            }
            int one = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            if (!options.quiet) std::cout << "[control] client connected\n";
            handleControlClient(clientFd, shared, options.quiet);
            ::shutdown(clientFd, SHUT_RDWR);
            ::close(clientFd);
            if (!options.quiet) std::cout << "[control] client disconnected\n";
        }
        ::close(serverFd);
    } catch (const std::exception& ex) {
        std::cerr << "control server error: " << ex.what() << "\n";
        g_running.store(false);
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::vector<std::string> args(argv, argv + argc);
        for (const std::string& arg : args) {
            if (arg == "--help" || arg == "-h") {
                usage(argv[0]);
                return 0;
            }
        }

        bool explicitConfigPath = false;
        const std::string configPath = virtual_bridge::findConfigPathArgument(
            args, virtual_bridge::defaultConfigPath(), explicitConfigPath);
        virtual_bridge::AppConfig options;
        if (virtual_bridge::appConfigFileExists(configPath)) {
            options = virtual_bridge::loadAppConfigFile(configPath);
        } else if (explicitConfigPath) {
            throw std::runtime_error("config file not found: " + configPath);
        } else {
            options.configPath = configPath;
        }
        virtual_bridge::applyCommandLineOverrides(options, args);

        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);

        SharedCommand shared;
        shared.command.servoPulseUs = static_cast<std::uint16_t>(options.vehicle.servoMidPulseUs);

        sockaddr_in udpTarget{};
        const int udpFd = createUdpSocket(options.udpHost, options.udpPort, udpTarget);

        virtual_bridge::VehicleModel model(options.vehicle);
        model.resetFromPosePoint(
            options.initialWorldXmm * 0.001,
            options.initialWorldYmm * 0.001,
            options.initialHeadingDeg);

        if (!options.quiet) {
            std::cout << "virtual_aruco_bridge udp_robot_position="
                      << options.udpHost << ":" << options.udpPort
                      << " send_hz=" << options.sendHz << "\n"
                      << "config=" << options.configPath << "\n"
                      << "initial_pose_point world_x_mm=" << options.initialWorldXmm
                      << " world_y_mm=" << options.initialWorldYmm
                      << " heading_deg=" << options.initialHeadingDeg << "\n"
                      << "vehicle wheelbase_m=" << options.vehicle.wheelbaseM
                      << " rear_track_m=" << options.vehicle.rearTrackM
                      << " pose_offset_m=" << options.vehicle.posePointForwardOffsetM << "\n";
        }

        std::thread controlThread(controlServerLoop, options, std::ref(shared));

        const double sendHz = options.sendHz > 0.0 ? options.sendHz : 30.0;
        const auto frameDuration = std::chrono::duration<double>(1.0 / sendHz);
        auto last = std::chrono::steady_clock::now();
        auto nextStatus = last + std::chrono::seconds(1);

        while (g_running.load()) {
            const auto now = std::chrono::steady_clock::now();
            const double dtS = std::chrono::duration<double>(now - last).count();
            last = now;

            const auto [command, commandFrames] = latestCommand(shared);
            model.update(command, dtS);
            const virtual_bridge::PosePoint pose = model.posePoint();
            const std::string payload =
                virtual_bridge::buildRobotPositionJson(pose, options.robotPosition, monotonicNowNs());
            const ssize_t sent = ::sendto(
                udpFd,
                payload.data(),
                payload.size(),
                0,
                reinterpret_cast<const sockaddr*>(&udpTarget),
                sizeof(udpTarget));
            if (sent < 0 && !options.quiet) {
                std::cerr << "udp send failed: " << std::strerror(errno) << "\n";
            }

            if (!options.quiet && now >= nextStatus) {
                const virtual_bridge::RearAxlePose rear = model.rearAxlePose();
                std::cout << std::fixed << std::setprecision(3)
                          << "[pose] point_x_m=" << pose.worldXM
                          << " point_y_m=" << pose.worldYM
                          << " heading_deg=" << pose.headingDeg
                          << " rear_speed_mps=" << rear.speedMps
                          << " steering_deg=" << virtual_bridge::radiansToDegrees(rear.steeringRad)
                          << " command_frames=" << commandFrames << "\n";
                nextStatus = now + std::chrono::seconds(1);
            }

            std::this_thread::sleep_until(now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(frameDuration));
        }

        ::close(udpFd);
        if (controlThread.joinable()) controlThread.join();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "virtual_aruco_bridge error: " << ex.what() << "\n";
        return 1;
    }
}
