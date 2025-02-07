#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>
#include "../annoy/src/annoylib.h"
#include "../annoy/src/kissrandom.h"

using namespace Annoy;

struct Point {
    double x, y;
};

struct Odometry {
    Point pose;
    double yaw;
};

struct Waypoint {
    Point pose;
    double tangent;
};

Point subtract(const Point &a, const Point &b) {
    return {a.x - b.x, a.y - b.y};
}

std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Odometry& odom) {
    os << "Pose: " << odom.pose << ", Yaw: " << odom.yaw;
    return os;
}

double distance(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

Point normalize(const Point &p) {
    double len = std::sqrt(p.x * p.x + p.y * p.y);
    if (len == 0) return {0, 0};
    return {p.x / len, p.y / len};
}

struct PointCloud {
    std::vector<Point> pts;
};

Odometry get_odometry(std::string odometry_csv) {
    std::ifstream file(odometry_csv);
    if (!file) {
        std::cerr << "Error 1\n";
        return {};
    }
    std::string line;
    Odometry odometry;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, yaw_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, yaw_str, ',');
        if (!x_str.empty() && !y_str.empty() && !yaw_str.empty() && x_str != "x") {
            odometry.pose.x = std::stod(x_str);
            odometry.pose.y = std::stod(y_str);
            odometry.yaw = std::stod(yaw_str);
        }
    }

    file.close();
    return odometry;
}

PointCloud get_trajectory(std::string trajectory_csv) {
    std::ifstream file(trajectory_csv);
    std::string line;
    PointCloud cloud;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        if (!x_str.empty() && !y_str.empty() && x_str != "x") {
            cloud.pts.push_back({std::stod(x_str), std::stod(y_str)});
        }
    }
    file.close();
    return cloud;
}

size_t get_closest_point(PointCloud& cloud, Point odometry_pose) {
    AnnoyIndex<int, double, Euclidean, Kiss32Random, AnnoyIndexSingleThreadedBuildPolicy> tree(2);

    // Inserisce i punti nella struttura Annoy
    for (size_t i = 0; i < cloud.pts.size(); ++i) {
        double data[2] = {cloud.pts[i].x, cloud.pts[i].y};
        tree.add_item(i, data);
    }

    tree.build(10);  // Crea l'albero con 10 alberi per la ricerca approssimata

    // Punto di query
    double query_point[2] = {odometry_pose.x, odometry_pose.y};

    std::vector<int> result;
    std::vector<double> distances;
    tree.get_nns_by_vector(query_point, 1, -1, &result, &distances);

    return result[0];  // Restituisce l'indice del punto più vicino
}

std::vector<double> get_tangent_angles(std::vector<Point> points) {
    std::vector<double> tangent_angles(points.size());

    if (points.size() >= 2) {
        Point diff = subtract(points[1], points[0]);
        Point tanVec = normalize(diff);
        tangent_angles[0] = std::atan2(tanVec.y, tanVec.x);

        diff = subtract(points.back(), points[points.size() - 2]);
        tanVec = normalize(diff);
        tangent_angles.back() = std::atan2(tanVec.y, tanVec.x);
    }

    for (size_t i = 1; i < points.size() - 1; ++i) {
        double d1 = distance(points[i], points[i - 1]);
        double d2 = distance(points[i + 1], points[i]);
        double ds = d1 + d2;

        if (ds == 0) {
            tangent_angles[i] = 0;
        } else {
            Point diff = {
                (points[i + 1].x - points[i - 1].x) / ds,
                (points[i + 1].y - points[i - 1].y) / ds
            };
            Point tanVec = normalize(diff);
            tangent_angles[i] = std::atan2(tanVec.y, tanVec.x);
        }
    }

    return tangent_angles;
}

double get_angular_deviation(double angle1, double angle2) {
    double diff = angle2 - angle1 + M_PI;
    diff = std::fmod(diff, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    diff -= M_PI;
    return std::abs(diff);
}

int main(int argc, char* argv[]) {
    std::string odometry_csv = "odometry/odometry.csv";
    std::string trajectory_csv = "trajectories/trajectory_downsampled.csv";
    if (argc > 2) {
        odometry_csv = argv[1];
        trajectory_csv = argv[2];
        std::cout << "[BENCHMARK]: Custom files loaded\n";
    }

    Odometry odometry = get_odometry(odometry_csv);
    Point odometry_pose = odometry.pose;
    PointCloud cloud = get_trajectory(trajectory_csv);

    auto start = std::chrono::high_resolution_clock::now();

    size_t closest_point_index = get_closest_point(cloud, odometry_pose);
    Point closest_point = cloud.pts[closest_point_index];

    double lateral_deviation = distance(odometry_pose, closest_point);

    std::vector<double> points_tangents = get_tangent_angles(cloud.pts);
    double closest_point_tangent = points_tangents[closest_point_index];

    auto end = std::chrono::high_resolution_clock::now();
    double time_taken = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

    std::cout << "Lateral deviation: " << lateral_deviation << std::endl;
    std::cout << "Angular deviation: " << get_angular_deviation(closest_point_tangent, odometry.yaw) << std::endl;
    std::cout << "[Time]: " << time_taken << " milliseconds" << std::endl;

    return 0;
}
