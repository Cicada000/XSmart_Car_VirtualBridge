#include "virtual_bridge/AppConfig.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace virtual_bridge {
namespace {

struct JsonValue {
    enum class Type {
        Null,
        Bool,
        Number,
        String,
        Array,
        Object
    };

    Type type = Type::Null;
    bool boolValue = false;
    double numberValue = 0.0;
    std::string stringValue;
    std::vector<JsonValue> arrayValue;
    std::map<std::string, JsonValue> objectValue;

    const JsonValue* member(const std::string& name) const {
        if (type != Type::Object) return nullptr;
        const auto it = objectValue.find(name);
        return it == objectValue.end() ? nullptr : &it->second;
    }
};

std::string readTextFile(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("cannot open config: " + path);
    }
    std::ostringstream out;
    out << input.rdbuf();
    return out.str();
}

std::string stripJsonComments(const std::string& input) {
    std::string output;
    output.reserve(input.size());
    bool inString = false;
    bool escaped = false;

    for (std::size_t i = 0; i < input.size(); ++i) {
        const char c = input[i];
        if (inString) {
            output.push_back(c);
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                inString = false;
            }
            continue;
        }

        if (c == '"') {
            inString = true;
            output.push_back(c);
            continue;
        }
        if (c == '/' && i + 1 < input.size() && input[i + 1] == '/') {
            i += 2;
            while (i < input.size() && input[i] != '\n') ++i;
            if (i < input.size()) output.push_back('\n');
            continue;
        }
        if (c == '/' && i + 1 < input.size() && input[i + 1] == '*') {
            i += 2;
            while (i + 1 < input.size() && !(input[i] == '*' && input[i + 1] == '/')) ++i;
            if (i + 1 < input.size()) ++i;
            continue;
        }
        output.push_back(c);
    }
    return output;
}

class JsonParser {
public:
    explicit JsonParser(std::string text) : text_(std::move(text)) {}

    JsonValue parse() {
        JsonValue value = parseValue();
        skipWhitespace();
        if (pos_ != text_.size()) {
            throw std::runtime_error("unexpected trailing config text");
        }
        return value;
    }

private:
    void skipWhitespace() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) {
            ++pos_;
        }
    }

    char peek() const {
        return pos_ < text_.size() ? text_[pos_] : '\0';
    }

    char take() {
        if (pos_ >= text_.size()) throw std::runtime_error("unexpected end of config");
        return text_[pos_++];
    }

    void expect(char expected) {
        const char actual = take();
        if (actual != expected) {
            throw std::runtime_error(std::string("expected '") + expected + "' in config");
        }
    }

    JsonValue parseValue() {
        skipWhitespace();
        const char c = peek();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == '"') return parseStringValue();
        if (c == '-' || c == '+' || std::isdigit(static_cast<unsigned char>(c))) return parseNumber();
        if (matchLiteral("true")) {
            JsonValue value;
            value.type = JsonValue::Type::Bool;
            value.boolValue = true;
            return value;
        }
        if (matchLiteral("false")) {
            JsonValue value;
            value.type = JsonValue::Type::Bool;
            value.boolValue = false;
            return value;
        }
        if (matchLiteral("null")) {
            return {};
        }
        throw std::runtime_error("invalid config value");
    }

    bool matchLiteral(const char* literal) {
        const std::string word(literal);
        if (text_.compare(pos_, word.size(), word) != 0) return false;
        pos_ += word.size();
        return true;
    }

    JsonValue parseObject() {
        JsonValue value;
        value.type = JsonValue::Type::Object;
        expect('{');
        skipWhitespace();
        if (peek() == '}') {
            take();
            return value;
        }

        while (true) {
            skipWhitespace();
            JsonValue key = parseStringValue();
            skipWhitespace();
            expect(':');
            value.objectValue[key.stringValue] = parseValue();
            skipWhitespace();
            const char separator = take();
            if (separator == '}') break;
            if (separator != ',') throw std::runtime_error("expected ',' or '}' in config object");
        }
        return value;
    }

    JsonValue parseArray() {
        JsonValue value;
        value.type = JsonValue::Type::Array;
        expect('[');
        skipWhitespace();
        if (peek() == ']') {
            take();
            return value;
        }

        while (true) {
            value.arrayValue.push_back(parseValue());
            skipWhitespace();
            const char separator = take();
            if (separator == ']') break;
            if (separator != ',') throw std::runtime_error("expected ',' or ']' in config array");
        }
        return value;
    }

    JsonValue parseStringValue() {
        JsonValue value;
        value.type = JsonValue::Type::String;
        expect('"');
        while (true) {
            const char c = take();
            if (c == '"') break;
            if (c == '\\') {
                const char esc = take();
                switch (esc) {
                case '"': value.stringValue.push_back('"'); break;
                case '\\': value.stringValue.push_back('\\'); break;
                case '/': value.stringValue.push_back('/'); break;
                case 'b': value.stringValue.push_back('\b'); break;
                case 'f': value.stringValue.push_back('\f'); break;
                case 'n': value.stringValue.push_back('\n'); break;
                case 'r': value.stringValue.push_back('\r'); break;
                case 't': value.stringValue.push_back('\t'); break;
                default:
                    throw std::runtime_error("unsupported string escape in config");
                }
            } else {
                value.stringValue.push_back(c);
            }
        }
        return value;
    }

    JsonValue parseNumber() {
        const std::size_t start = pos_;
        if (peek() == '+' || peek() == '-') ++pos_;
        while (std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        if (peek() == '.') {
            ++pos_;
            while (std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        }
        if (peek() == 'e' || peek() == 'E') {
            ++pos_;
            if (peek() == '+' || peek() == '-') ++pos_;
            while (std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
        }

        char* end = nullptr;
        const std::string token = text_.substr(start, pos_ - start);
        const double number = std::strtod(token.c_str(), &end);
        if (end == token.c_str() || *end != '\0') {
            throw std::runtime_error("invalid config number: " + token);
        }
        JsonValue value;
        value.type = JsonValue::Type::Number;
        value.numberValue = number;
        return value;
    }

    std::string text_;
    std::size_t pos_ = 0;
};

JsonValue parseConfigText(const std::string& text) {
    JsonParser parser(stripJsonComments(text));
    return parser.parse();
}

const JsonValue* objectMember(const JsonValue& root, const std::string& name) {
    return root.member(name);
}

std::string getString(const JsonValue* object, const std::string& key, const std::string& fallback) {
    if (!object) return fallback;
    const JsonValue* value = object->member(key);
    return value && value->type == JsonValue::Type::String ? value->stringValue : fallback;
}

double getNumber(const JsonValue* object, const std::string& key, double fallback) {
    if (!object) return fallback;
    const JsonValue* value = object->member(key);
    return value && value->type == JsonValue::Type::Number ? value->numberValue : fallback;
}

int getInt(const JsonValue* object, const std::string& key, int fallback) {
    return static_cast<int>(getNumber(object, key, static_cast<double>(fallback)));
}

bool getBool(const JsonValue* object, const std::string& key, bool fallback) {
    if (!object) return fallback;
    const JsonValue* value = object->member(key);
    return value && value->type == JsonValue::Type::Bool ? value->boolValue : fallback;
}

CoordinateSource coordinateSourceFromString(const std::string& text, const char* fieldName) {
    if (text == "world_x" || text == "x") return CoordinateSource::WorldX;
    if (text == "world_y" || text == "y") return CoordinateSource::WorldY;
    throw std::runtime_error(std::string(fieldName) + " must be world_x or world_y: " + text);
}

CoordinateSource getCoordinateSource(const JsonValue* object,
                                     const std::string& key,
                                     CoordinateSource fallback) {
    if (!object) return fallback;
    const JsonValue* value = object->member(key);
    if (!value) return fallback;
    if (value->type != JsonValue::Type::String) {
        throw std::runtime_error("config field " + key + " must be a string");
    }
    return coordinateSourceFromString(value->stringValue, key.c_str());
}

int parseIntArg(const std::string& text, const char* name) {
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0') {
        throw std::runtime_error(std::string("invalid ") + name + ": " + text);
    }
    return static_cast<int>(value);
}

double parseDoubleArg(const std::string& text, const char* name) {
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') {
        throw std::runtime_error(std::string("invalid ") + name + ": " + text);
    }
    return value;
}

} // namespace

std::string defaultConfigPath() {
    return "config/virtual_bridge.json";
}

bool appConfigFileExists(const std::string& path) {
    std::ifstream input(path);
    return input.good();
}

AppConfig loadAppConfigFile(const std::string& path) {
    AppConfig config;
    config.configPath = path;
    const JsonValue root = parseConfigText(readTextFile(path));
    if (root.type != JsonValue::Type::Object) {
        throw std::runtime_error("config root must be an object: " + path);
    }

    const JsonValue* control = objectMember(root, "control");
    config.controlBindIp = getString(control, "bind_ip", config.controlBindIp);
    config.controlPort = getInt(control, "port", config.controlPort);

    const JsonValue* udp = objectMember(root, "udp");
    config.udpHost = getString(udp, "host", config.udpHost);
    config.udpPort = getInt(udp, "port", config.udpPort);
    config.sendHz = getNumber(udp, "send_hz", config.sendHz);

    const JsonValue* initialPose = objectMember(root, "initial_pose");
    config.initialWorldXmm = getNumber(initialPose, "world_x_mm", config.initialWorldXmm);
    config.initialWorldYmm = getNumber(initialPose, "world_y_mm", config.initialWorldYmm);
    config.initialHeadingDeg = getNumber(initialPose, "heading_deg", config.initialHeadingDeg);

    const JsonValue* vehicle = objectMember(root, "vehicle");
    config.vehicle.wheelbaseM = getNumber(vehicle, "wheelbase_m", config.vehicle.wheelbaseM);
    config.vehicle.rearTrackM = getNumber(vehicle, "rear_track_m", config.vehicle.rearTrackM);
    config.vehicle.posePointForwardOffsetM = getNumber(vehicle, "pose_offset_m", config.vehicle.posePointForwardOffsetM);
    config.vehicle.servoMidPulseUs = getInt(vehicle, "servo_mid_us", config.vehicle.servoMidPulseUs);
    config.vehicle.servoPulseSpanUs = getNumber(vehicle, "servo_span_us", config.vehicle.servoPulseSpanUs);
    config.vehicle.maxSteeringDeg = getNumber(vehicle, "max_steering_deg", config.vehicle.maxSteeringDeg);
    config.vehicle.steeringSign = getNumber(vehicle, "steering_sign", config.vehicle.steeringSign);
    config.vehicle.servoSecPer60Deg = getNumber(vehicle, "servo_sec_per_60_deg", config.vehicle.servoSecPer60Deg);
    config.vehicle.speedScale = getNumber(vehicle, "speed_scale", config.vehicle.speedScale);
    config.vehicle.speedTimeConstantS = getNumber(vehicle, "speed_tau_s", config.vehicle.speedTimeConstantS);
    config.vehicle.maxAccelMps2 = getNumber(vehicle, "max_accel_mps2", config.vehicle.maxAccelMps2);
    config.vehicle.clampNegativeSpeed = getBool(vehicle, "clamp_negative_speed", config.vehicle.clampNegativeSpeed);
    config.vehicle.maxDtS = getNumber(vehicle, "max_dt_s", config.vehicle.maxDtS);

    const JsonValue* physics = objectMember(root, "physics_enhancements");
    if (!physics) physics = objectMember(root, "physics");
    config.vehicle.physics.enabled = getBool(physics, "enabled", config.vehicle.physics.enabled);
    config.vehicle.physics.vehicleMassKg = getNumber(physics, "vehicle_mass_kg", config.vehicle.physics.vehicleMassKg);
    config.vehicle.physics.minStartSpeedMps = getNumber(physics, "min_start_speed_mps", config.vehicle.physics.minStartSpeedMps);
    config.vehicle.physics.coastingDecelMps2 = getNumber(physics, "coasting_decel_mps2", config.vehicle.physics.coastingDecelMps2);
    config.vehicle.physics.servoTrimUs = getInt(physics, "servo_trim_us", config.vehicle.physics.servoTrimUs);
    config.vehicle.physics.servoDeadbandUs = getNumber(physics, "servo_deadband_us", config.vehicle.physics.servoDeadbandUs);
    config.vehicle.physics.sensorLatencyMs = getNumber(physics, "sensor_latency_ms", config.vehicle.physics.sensorLatencyMs);
    config.vehicle.physics.positionNoiseM = getNumber(physics, "position_noise_m", config.vehicle.physics.positionNoiseM);
    config.vehicle.physics.yawNoiseDeg = getNumber(physics, "yaw_noise_deg", config.vehicle.physics.yawNoiseDeg);

    const JsonValue* robotPosition = objectMember(root, "robot_position");
    config.robotPosition.heightM = getNumber(robotPosition, "height_m", config.robotPosition.heightM);
    config.robotPosition.posXSource = getCoordinateSource(
        robotPosition, "pos_x_source", config.robotPosition.posXSource);
    config.robotPosition.posXSign = getNumber(robotPosition, "pos_x_sign", config.robotPosition.posXSign);
    config.robotPosition.posZSource = getCoordinateSource(
        robotPosition, "pos_z_source", config.robotPosition.posZSource);
    config.robotPosition.posZSign = getNumber(robotPosition, "pos_z_sign", config.robotPosition.posZSign);
    config.robotPosition.yawOffsetDeg = getNumber(robotPosition, "yaw_offset_deg", config.robotPosition.yawOffsetDeg);
    config.robotPosition.yawSign = getNumber(robotPosition, "yaw_sign", config.robotPosition.yawSign);

    const JsonValue* runtime = objectMember(root, "runtime");
    config.quiet = getBool(runtime, "quiet", config.quiet);
    return config;
}

void applyCommandLineOverrides(AppConfig& config, const std::vector<std::string>& args) {
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        auto need = [&](const char* name) -> const std::string& {
            if (i + 1 >= args.size()) throw std::runtime_error(std::string("missing value for ") + name);
            return args[++i];
        };

        if (arg == "--config") {
            config.configPath = need("--config");
        } else if (arg == "--help" || arg == "-h") {
            continue;
        } else if (arg == "--control-bind" || arg == "--bind") {
            config.controlBindIp = need(arg.c_str());
        } else if (arg == "--control-port" || arg == "--port") {
            config.controlPort = parseIntArg(need(arg.c_str()), "control port");
        } else if (arg == "--udp-host") {
            config.udpHost = need("--udp-host");
        } else if (arg == "--udp-port") {
            config.udpPort = parseIntArg(need("--udp-port"), "udp port");
        } else if (arg == "--send-hz") {
            config.sendHz = parseDoubleArg(need("--send-hz"), "send hz");
        } else if (arg == "--initial-world-x-mm") {
            config.initialWorldXmm = parseDoubleArg(need("--initial-world-x-mm"), "initial world x mm");
        } else if (arg == "--initial-world-y-mm") {
            config.initialWorldYmm = parseDoubleArg(need("--initial-world-y-mm"), "initial world y mm");
        } else if (arg == "--initial-heading-deg") {
            config.initialHeadingDeg = parseDoubleArg(need("--initial-heading-deg"), "initial heading deg");
        } else if (arg == "--wheelbase-m") {
            config.vehicle.wheelbaseM = parseDoubleArg(need("--wheelbase-m"), "wheelbase");
        } else if (arg == "--rear-track-m") {
            config.vehicle.rearTrackM = parseDoubleArg(need("--rear-track-m"), "rear track");
        } else if (arg == "--pose-offset-m") {
            config.vehicle.posePointForwardOffsetM = parseDoubleArg(need("--pose-offset-m"), "pose offset");
        } else if (arg == "--servo-mid-us") {
            config.vehicle.servoMidPulseUs = parseIntArg(need("--servo-mid-us"), "servo mid");
        } else if (arg == "--servo-span-us") {
            config.vehicle.servoPulseSpanUs = parseDoubleArg(need("--servo-span-us"), "servo span");
        } else if (arg == "--max-steering-deg") {
            config.vehicle.maxSteeringDeg = parseDoubleArg(need("--max-steering-deg"), "max steering");
        } else if (arg == "--steering-sign") {
            config.vehicle.steeringSign = parseDoubleArg(need("--steering-sign"), "steering sign");
        } else if (arg == "--servo-sec-per-60-deg") {
            config.vehicle.servoSecPer60Deg = parseDoubleArg(need("--servo-sec-per-60-deg"), "servo speed");
        } else if (arg == "--speed-scale") {
            config.vehicle.speedScale = parseDoubleArg(need("--speed-scale"), "speed scale");
        } else if (arg == "--speed-tau-s") {
            config.vehicle.speedTimeConstantS = parseDoubleArg(need("--speed-tau-s"), "speed tau");
        } else if (arg == "--max-accel-mps2") {
            config.vehicle.maxAccelMps2 = parseDoubleArg(need("--max-accel-mps2"), "max accel");
        } else if (arg == "--height-m") {
            config.robotPosition.heightM = parseDoubleArg(need("--height-m"), "height");
        } else if (arg == "--yaw-offset-deg") {
            config.robotPosition.yawOffsetDeg = parseDoubleArg(need("--yaw-offset-deg"), "yaw offset");
        } else if (arg == "--yaw-sign") {
            config.robotPosition.yawSign = parseDoubleArg(need("--yaw-sign"), "yaw sign");
        } else if (arg == "--pos-x-source") {
            config.robotPosition.posXSource = coordinateSourceFromString(
                need("--pos-x-source"), "--pos-x-source");
        } else if (arg == "--pos-x-sign") {
            config.robotPosition.posXSign = parseDoubleArg(need("--pos-x-sign"), "pos x sign");
        } else if (arg == "--pos-z-source") {
            config.robotPosition.posZSource = coordinateSourceFromString(
                need("--pos-z-source"), "--pos-z-source");
        } else if (arg == "--pos-z-sign") {
            config.robotPosition.posZSign = parseDoubleArg(need("--pos-z-sign"), "pos z sign");
        } else if (arg == "--clamp-negative-speed") {
            config.vehicle.clampNegativeSpeed = true;
        } else if (arg == "--allow-negative-speed") {
            config.vehicle.clampNegativeSpeed = false;
        } else if (arg == "--max-dt-s") {
            config.vehicle.maxDtS = parseDoubleArg(need("--max-dt-s"), "max dt");
        } else if (arg == "--enable-physics") {
            config.vehicle.physics.enabled = true;
        } else if (arg == "--disable-physics") {
            config.vehicle.physics.enabled = false;
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
}

void applyCommandLineOverrides(AppConfig& config, int argc, const char* const* argv) {
    std::vector<std::string> args;
    args.reserve(static_cast<std::size_t>(argc));
    for (int i = 0; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }
    applyCommandLineOverrides(config, args);
}

std::string findConfigPathArgument(const std::vector<std::string>& args,
                                   const std::string& fallback,
                                   bool& explicitPath) {
    explicitPath = false;
    for (std::size_t i = 1; i < args.size(); ++i) {
        if (args[i] == "--config") {
            if (i + 1 >= args.size()) throw std::runtime_error("missing value for --config");
            explicitPath = true;
            return args[i + 1];
        }
    }
    return fallback;
}

} // namespace virtual_bridge
