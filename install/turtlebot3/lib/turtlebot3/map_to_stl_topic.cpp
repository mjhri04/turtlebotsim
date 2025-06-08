#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>

using std::placeholders::_1;

class MapToStlNode : public rclcpp::Node {
public:
  MapToStlNode() : Node("map_to_stl_node") {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MapToStlNode::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Waiting for /map...");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Map received. Generating STL...");
    generate_stl(*msg);
  }

  void generate_stl(const nav_msgs::msg::OccupancyGrid & map) {
    std::string path = "/home/ju/sim_map/map.stl";
    std::filesystem::create_directories("/home/ju/sim_map");
    std::ofstream stl(path);
    if (!stl.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open STL file for writing.");
      return;
    }

    stl << "solid map\n";
    float height = 1.0f;
    float res = map.info.resolution;
    int width = map.info.width;
    int height_px = map.info.height;

    for (int y = 0; y < height_px; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        if (map.data[idx] > 100) { // threshold for occupied
          float ox = x * res + map.info.origin.position.x;
          float oy = y * res + map.info.origin.position.y;
          write_box(stl, ox, oy, 0.0f, res, res, height);
        }
      }
    }

    stl << "endsolid map\n";
    stl.close();
    RCLCPP_INFO(this->get_logger(), "STL saved to %s", path.c_str());
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