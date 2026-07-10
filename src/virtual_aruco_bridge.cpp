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

struct Options {
    std::string controlBindIp = "127.0.0.1";
    int controlPort = 8899;
    std::string udpHost = "192.168.8.123";
    int udpPort = 9005;
    double sendHz = 30.0;
    double initialWorldXmm = 0.0;
    double initialWorldYmm = 0.0;
    double initialHeadingDeg = 0.0;
    bool quiet = false;
    virtual_bridge::VehicleParameters vehicle;
    virtual_bridge::RobotPositionConfig robotPosition;
};

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
        << "  --control-bind IP            TCP bind IP for XSmart control frames, default 127.0.0.1\n"
        << "  --control-port PORT          TCP control port, default 8899\n"
        << "  --udp-host IP                UDP robot_position target, default 192.168.8.123\n"
        << "  --udp-port PORT              UDP robot_position target port, default 9005\n"
        << "  --send-hz HZ                 UDP publish frequency, default 30\n"
        << "  --initial-world-x-mm MM      Initial localization point world X in ArUco field coordinates\n"
        << "  --initial-world-y-mm MM      Initial localization point world Y in ArUco field coordinates\n"
        << "  --initial-heading-deg DEG    Initial vehicle heading in ArUco world XY plane\n"
        << "  --wheelbase-m M              Front/rear axle distance, default 0.20\n"
        << "  --rear-track-m M             Left/right rear wheel distance, default 0.155\n"
        << "  --pose-offset-m M            Localization point offset from rear axle center, default 0.075\n"
        << "  --servo-mid-us US            Neutral servo pulse, default 1500\n"
        << "  --servo-span-us US           Pulse span for 180 degrees, default 2000\n"
        << "  --max-steering-deg DEG       Clamp front wheel angle, default 36\n"
        << "  --steering-sign SIGN         Steering polarity, 1 or -1, default 1\n"
        << "  --servo-sec-per-60-deg S     Servo speed model, default 0.16\n"
        << "  --speed-scale SCALE          Multiplier from command speed to simulated m/s, default 1\n"
        << "  --speed-tau-s S              First-order speed response time constant, default 0.12\n"
        << "  --max-accel-mps2 A           Acceleration clamp, default 3\n"
        << "  --height-m M                 robot_position Y height, default 0.16\n"
        << "  --yaw-offset-deg DEG         robot_position yaw offset, default 0\n"
        << "  --yaw-sign SIGN              robot_position yaw sign, default 1\n"
        << "  --pos-x-source world_x|world_y  Source for robot_position pos[0], default world_y\n"
        << "  --pos-z-source world_x|world_y  Source for robot_position pos[2], default world_x\n"
        << "  --quiet                      Suppress periodic status output\n"
        << "  -h, --help                   Show help\n";
}

int parseInt(const std::string& text, const char* name) {
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0') {
        throw std::runtime_error(std::string("invalid ") + name + ": " + text);
    }
    return static_cast<int>(value);
}

double parseDouble(const std::string& text, const char* name) {
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') {
        throw std::runtime_error(std::string("invalid ") + name + ": " + text);
    }
    return value;
}

virtual_bridge::CoordinateSource parseCoordinateSource(const std::string& text) {
    if (text == "world_x" || text == "x") return virtual_bridge::CoordinateSource::WorldX;
    if (text == "world_y" || text == "y") return virtual_bridge::CoordinateSource::WorldY;
    throw std::runtime_error("coordinate source must be world_x or world_y: " + text);
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else if (arg == "--control-bind" || arg == "--bind") {
            options.controlBindIp = need(arg.c_str());
        } else if (arg == "--control-port" || arg == "--port") {
            options.controlPort = parseInt(need(arg.c_str()), "control port");
        } else if (arg == "--udp-host") {
            options.udpHost = need("--udp-host");
        } else if (arg == "--udp-port") {
            options.udpPort = parseInt(need("--udp-port"), "udp port");
        } else if (arg == "--send-hz") {
            options.sendHz = parseDouble(need("--send-hz"), "send hz");
        } else if (arg == "--initial-world-x-mm") {
            options.initialWorldXmm = parseDouble(need("--initial-world-x-mm"), "initial world x mm");
        } else if (arg == "--initial-world-y-mm") {
            options.initialWorldYmm = parseDouble(need("--initial-world-y-mm"), "initial world y mm");
        } else if (arg == "--initial-heading-deg") {
            options.initialHeadingDeg = parseDouble(need("--initial-heading-deg"), "initial heading deg");
        } else if (arg == "--wheelbase-m") {
            options.vehicle.wheelbaseM = parseDouble(need("--wheelbase-m"), "wheelbase");
        } else if (arg == "--rear-track-m") {
            options.vehicle.rearTrackM = parseDouble(need("--rear-track-m"), "rear track");
        } else if (arg == "--pose-offset-m") {
            options.vehicle.posePointForwardOffsetM = parseDouble(need("--pose-offset-m"), "pose offset");
        } else if (arg == "--servo-mid-us") {
            options.vehicle.servoMidPulseUs = parseInt(need("--servo-mid-us"), "servo mid");
        } else if (arg == "--servo-span-us") {
            options.vehicle.servoPulseSpanUs = parseDouble(need("--servo-span-us"), "servo span");
        } else if (arg == "--max-steering-deg") {
            options.vehicle.maxSteeringDeg = parseDouble(need("--max-steering-deg"), "max steering");
        } else if (arg == "--steering-sign") {
            options.vehicle.steeringSign = parseDouble(need("--steering-sign"), "steering sign");
        } else if (arg == "--servo-sec-per-60-deg") {
            options.vehicle.servoSecPer60Deg = parseDouble(need("--servo-sec-per-60-deg"), "servo speed");
        } else if (arg == "--speed-scale") {
            options.vehicle.speedScale = parseDouble(need("--speed-scale"), "speed scale");
        } else if (arg == "--speed-tau-s") {
            options.vehicle.speedTimeConstantS = parseDouble(need("--speed-tau-s"), "speed tau");
        } else if (arg == "--max-accel-mps2") {
            options.vehicle.maxAccelMps2 = parseDouble(need("--max-accel-mps2"), "max accel");
        } else if (arg == "--height-m") {
            options.robotPosition.heightM = parseDouble(need("--height-m"), "height");
        } else if (arg == "--yaw-offset-deg") {
            options.robotPosition.yawOffsetDeg = parseDouble(need("--yaw-offset-deg"), "yaw offset");
        } else if (arg == "--yaw-sign") {
            options.robotPosition.yawSign = parseDouble(need("--yaw-sign"), "yaw sign");
        } else if (arg == "--pos-x-source") {
            options.robotPosition.posXSource = parseCoordinateSource(need("--pos-x-source"));
        } else if (arg == "--pos-z-source") {
            options.robotPosition.posZSource = parseCoordinateSource(need("--pos-z-source"));
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    return options;
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

void controlServerLoop(const Options& options, SharedCommand& shared) {
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
        Options options = parseArgs(argc, argv);
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
