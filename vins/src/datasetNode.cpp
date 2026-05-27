#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "estimator/estimator.h"
#include "utility/visualization.h"

namespace fs = std::filesystem;

Estimator estimator;

namespace {

struct Options {
    std::string vins_config;
    std::string preset;
    std::string data_path;
    std::string left_folder;
    std::string right_folder;
    std::string times_file;
    std::string imu_file;
    std::string image_ext = ".png";
    double time_scale = 1.0;
};

struct ImuSample {
    double t;
    Eigen::Vector3d gyr;
    Eigen::Vector3d acc;
};

void printUsage()
{
    std::fprintf(stderr,
        "Usage:\n"
        "  dataset_node <vins_config.yaml> --preset <name> --data <folder>\n"
        "  dataset_node <vins_config.yaml> --left <folder> --right <folder>\n"
        "                                 [--times <file>] [--imu <csv>]\n"
        "                                 [--ext <.png>] [--time-scale <factor>]\n"
        "\n"
        "Presets:\n"
        "  kitti_odom  image_0/ + image_1/ + times.txt (no IMU)\n"
        "  euroc       mav0/cam0/data + mav0/cam1/data + mav0/imu0/data.csv\n"
        "  tum_vi      mav0/cam0/data + mav0/cam1/data + mav0/imu0/data.csv\n"
        "\n"
        "Notes:\n"
        "  --time-scale converts file-encoded timestamps to seconds, and is\n"
        "    applied to image filenames AND the IMU CSV first column.\n"
        "    1.0 = seconds (default), 1e-9 = nanoseconds (EuRoC/TUM-VI).\n"
        "  --imu expects EuRoC-style CSV: t, wx, wy, wz, ax, ay, az.\n"
        "    For IMU fusion to actually run, the vins config must set imu: 1\n"
        "    with valid IMU intrinsics and cam-IMU extrinsics.\n"
        "  Stereo only. Images must be rectified and time-synced per pair.\n");
}

bool parseArgs(int argc, char** argv, Options& opts)
{
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") { printUsage(); return false; }
    }
    if (argc < 2) { printUsage(); return false; }
    opts.vins_config = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", flag);
                return nullptr;
            }
            return argv[++i];
        };
        if      (a == "--preset")     { auto v = next("--preset");     if (!v) return false; opts.preset = v; }
        else if (a == "--data")       { auto v = next("--data");       if (!v) return false; opts.data_path = v; }
        else if (a == "--left")       { auto v = next("--left");       if (!v) return false; opts.left_folder = v; }
        else if (a == "--right")      { auto v = next("--right");      if (!v) return false; opts.right_folder = v; }
        else if (a == "--times")      { auto v = next("--times");      if (!v) return false; opts.times_file = v; }
        else if (a == "--imu")        { auto v = next("--imu");        if (!v) return false; opts.imu_file = v; }
        else if (a == "--ext")        { auto v = next("--ext");        if (!v) return false; opts.image_ext = v; }
        else if (a == "--time-scale") { auto v = next("--time-scale"); if (!v) return false; opts.time_scale = std::stod(v); }
        else if (a == "-h" || a == "--help") { printUsage(); return false; }
        else if (a == "--ros-args") { break; }
        else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            printUsage();
            return false;
        }
    }
    return true;
}

bool applyPreset(Options& opts)
{
    if (opts.preset.empty()) return true;
    if (opts.data_path.empty()) {
        std::fprintf(stderr, "--data is required when --preset is set\n");
        return false;
    }
    fs::path data = opts.data_path;
    if (opts.preset == "kitti_odom") {
        opts.left_folder  = (data / "image_0").string();
        opts.right_folder = (data / "image_1").string();
        opts.times_file   = (data / "times.txt").string();
    } else if (opts.preset == "euroc") {
        opts.left_folder  = (data / "mav0" / "cam0" / "data").string();
        opts.right_folder = (data / "mav0" / "cam1" / "data").string();
        opts.time_scale   = 1e-9;
        if (opts.imu_file.empty()) {
            auto p = data / "mav0" / "imu0" / "data.csv";
            if (fs::exists(p)) opts.imu_file = p.string();
        }
    } else if (opts.preset == "tum_vi") {
        opts.left_folder  = (data / "mav0" / "cam0" / "data").string();
        opts.right_folder = (data / "mav0" / "cam1" / "data").string();
        opts.times_file   = (data / "mav0" / "cam0" / "data.csv").string();
        opts.time_scale   = 1e-9;
        if (opts.imu_file.empty()) {
            auto p = data / "mav0" / "imu0" / "data.csv";
            if (fs::exists(p)) opts.imu_file = p.string();
        }
    } else {
        std::fprintf(stderr, "unknown preset: %s\n", opts.preset.c_str());
        return false;
    }
    return true;
}

std::vector<fs::path> listImages(const std::string& folder, const std::string& ext)
{
    std::vector<fs::path> files;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(folder, ec)) {
        if (!e.is_regular_file()) continue;
        if (e.path().extension() == ext) files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::vector<double> readTimesFile(const std::string& path, double scale)
{
    std::vector<double> times;
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open times file: %s\n", path.c_str());
        return times;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto comma = line.find(',');
        std::string token = (comma == std::string::npos) ? line : line.substr(0, comma);
        try {
            times.push_back(std::stod(token) * scale);
        } catch (const std::exception&) {
        }
    }
    return times;
}

std::vector<ImuSample> readImuCsv(const std::string& path, double scale)
{
    std::vector<ImuSample> out;
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open imu file: %s\n", path.c_str());
        return out;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::vector<double> vals;
        std::string tok;
        std::stringstream ss(line);
        while (std::getline(ss, tok, ',')) {
            try { vals.push_back(std::stod(tok)); } catch (...) { vals.clear(); break; }
        }
        if (vals.size() < 7) continue;
        ImuSample s;
        s.t = vals[0] * scale;
        s.gyr << vals[1], vals[2], vals[3];
        s.acc << vals[4], vals[5], vals[6];
        out.push_back(s);
    }
    std::sort(out.begin(), out.end(),
              [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
    return out;
}

std::vector<double> timesFromFilenames(const std::vector<fs::path>& files, double scale)
{
    std::vector<double> times;
    times.reserve(files.size());
    for (const auto& f : files) {
        try {
            times.push_back(std::stod(f.stem().string()) * scale);
        } catch (const std::exception&) {
            std::fprintf(stderr, "filename is not numeric: %s — use --times <file>\n",
                         f.filename().string().c_str());
            return {};
        }
    }
    return times;
}

}

int main(int argc, char** argv)
{
    Options opts;
    if (!parseArgs(argc, argv, opts)) return 1;
    if (!applyPreset(opts)) return 1;

    if (opts.left_folder.empty() || opts.right_folder.empty()) {
        std::fprintf(stderr, "must provide --left and --right, or --preset and --data\n");
        return 1;
    }

    auto left  = listImages(opts.left_folder,  opts.image_ext);
    auto right = listImages(opts.right_folder, opts.image_ext);
    if (left.empty() || right.empty()) {
        std::fprintf(stderr, "no %s files found in %s or %s\n",
                     opts.image_ext.c_str(), opts.left_folder.c_str(), opts.right_folder.c_str());
        return 1;
    }
    if (left.size() != right.size()) {
        std::fprintf(stderr, "left/right image count mismatch: %zu vs %zu\n",
                     left.size(), right.size());
        return 1;
    }

    std::vector<double> times = opts.times_file.empty()
        ? timesFromFilenames(left, opts.time_scale)
        : readTimesFile(opts.times_file, opts.time_scale);
    if (times.size() != left.size()) {
        std::fprintf(stderr, "timestamp count (%zu) does not match image count (%zu)\n",
                     times.size(), left.size());
        return 1;
    }

    std::vector<ImuSample> imu;
    if (!opts.imu_file.empty()) {
        imu = readImuCsv(opts.imu_file, opts.time_scale);
        if (imu.empty()) {
            std::fprintf(stderr, "imu file produced no samples: %s\n", opts.imu_file.c_str());
            return 1;
        }
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("vins_estimator");
    auto pubLeft  = node->create_publisher<sensor_msgs::msg::Image>("/leftImage", 1000);
    auto pubRight = node->create_publisher<sensor_msgs::msg::Image>("/rightImage", 1000);

    readParameters(opts.vins_config);
    estimator.setParameter();
    registerPub(node);

    std::FILE* outFile = std::fopen((OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
    if (!outFile) {
        std::fprintf(stderr, "output path does not exist: %s\n", OUTPUT_FOLDER.c_str());
    }

    std::printf("dataset_node: %zu frame pairs from %s + %s\n",
                left.size(), opts.left_folder.c_str(), opts.right_folder.c_str());
    if (!imu.empty()) {
        std::printf("dataset_node: %zu IMU samples from %s\n",
                    imu.size(), opts.imu_file.c_str());
    }

    size_t imu_idx = 0;
    for (size_t i = 0; i < left.size() && rclcpp::ok(); ++i) {
        while (imu_idx < imu.size() && imu[imu_idx].t <= times[i]) {
            const auto& s = imu[imu_idx++];
            estimator.inputIMU(s.t, s.acc, s.gyr);
        }
        cv::Mat imLeft  = cv::imread(left[i].string(),  cv::IMREAD_GRAYSCALE);
        cv::Mat imRight = cv::imread(right[i].string(), cv::IMREAD_GRAYSCALE);
        if (imLeft.empty() || imRight.empty()) {
            std::fprintf(stderr, "failed to read frame %zu: %s / %s\n",
                         i, left[i].string().c_str(), right[i].string().c_str());
            continue;
        }

        auto leftMsg  = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imLeft).toImageMsg();
        auto rightMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imRight).toImageMsg();
        leftMsg->header.stamp  = rclcpp::Time(static_cast<int64_t>(times[i] * 1e9));
        rightMsg->header.stamp = leftMsg->header.stamp;
        pubLeft->publish(*leftMsg);
        pubRight->publish(*rightMsg);

        estimator.inputImage(times[i], imLeft, imRight);

        Eigen::Matrix<double, 4, 4> pose;
        estimator.getPoseInWorldFrame(pose);
        if (outFile) {
            std::fprintf(outFile,
                "%f %f %f %f %f %f %f %f %f %f %f %f \n",
                pose(0,0), pose(0,1), pose(0,2), pose(0,3),
                pose(1,0), pose(1,1), pose(1,2), pose(1,3),
                pose(2,0), pose(2,1), pose(2,2), pose(2,3));
        }
    }

    if (outFile) std::fclose(outFile);
    rclcpp::shutdown();
    return 0;
}
