#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>

class MapToStlNode : public rclcpp::Node {
public:
  MapToStlNode() : Node("map_to_stl_node") {
    RCLCPP_INFO(this->get_logger(), "Generating STL from .pgm + .yaml...");

    std::string yaml_path = "/home/ju/map.yaml";
    std::string stl_path = "/home/ju/map.stl";

    try {
      generate_stl_from_yaml(yaml_path, stl_path);
      RCLCPP_INFO(this->get_logger(), "STL saved to %s", stl_path.c_str());
      rclcpp::shutdown();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
      rclcpp::shutdown();
    }
  }

private:
  void generate_stl_from_yaml(const std::string &yaml_file, const std::string &stl_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);

    std::string pgm_path = config["image"].as<std::string>();
    if (pgm_path.front() != '/') {
      pgm_path = std::filesystem::path(yaml_file).parent_path() / pgm_path;
    }

    double resolution = config["resolution"].as<double>();
    double origin_x = config["origin"][0].as<double>();
    double origin_y = config["origin"][1].as<double>();
    double origin_z = config["origin"][2].as<double>();

    cv::Mat img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) throw std::runtime_error("Failed to load PGM image.");

    std::filesystem::create_directories(std::filesystem::path(stl_file).parent_path());
    std::ofstream stl(stl_file);
    if (!stl.is_open()) throw std::runtime_error("Failed to open STL file for writing.");

    stl << "solid map\n";

    float height = 1.0f;
    int width = img.cols;
    int height_px = img.rows;

    for (int y = 0; y < height_px; ++y) {
      for (int x = 0; x < width; ++x) {
        uchar val = img.at<uchar>(y, x);
        if (val < 100) { // 0=occupied, 255=free
          float ox = x * resolution + origin_x;
          float oy = (height_px - y - 1) * resolution + origin_y;
          write_box(stl, ox, oy, origin_z, resolution, resolution, height);
        }
      }
    }

    stl << "endsolid map\n";
    stl.close();
  }

  void write_triangle(std::ofstream &stl, const std::array<float, 9> &v) {
    stl << "  facet normal 0 0 0\n";
    stl << "    outer loop\n";
    for (int i = 0; i < 3; ++i)
      stl << "      vertex " << v[i*3] << " " << v[i*3+1] << " " << v[i*3+2] << "\n";
    stl << "    endloop\n";
    stl << "  endfacet\n";
  }

  void write_box(std::ofstream &stl, float x, float y, float z, float dx, float dy, float dz) {
    float x0 = x,     x1 = x + dx;
    float y0 = y,     y1 = y + dy;
    float z0 = z,     z1 = z + dz;

    std::vector<std::array<float, 9>> triangles = {
      // bottom
      {x0,y0,z0, x1,y0,z0, x1,y1,z0}, {x0,y0,z0, x1,y1,z0, x0,y1,z0},
      // top
      {x0,y0,z1, x1,y1,z1, x1,y0,z1}, {x0,y0,z1, x0,y1,z1, x1,y1,z1},
      // front
      {x0,y0,z0, x0,y0,z1, x1,y0,z1}, {x0,y0,z0, x1,y0,z1, x1,y0,z0},
      // back
      {x0,y1,z0, x1,y1,z1, x0,y1,z1}, {x0,y1,z0, x1,y1,z0, x1,y1,z1},
      // left
      {x0,y0,z0, x0,y1,z1, x0,y0,z1}, {x0,y0,z0, x0,y1,z0, x0,y1,z1},
      // right
      {x1,y0,z0, x1,y0,z1, x1,y1,z1}, {x1,y0,z0, x1,y1,z1, x1,y1,z0}
    };

    for (const auto &tri : triangles)
      write_triangle(stl, tri);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToStlNode>());
  rclcpp::shutdown();
  return 0;
}
