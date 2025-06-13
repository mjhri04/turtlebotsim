#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <filesystem>

class MapToWorldNode : public rclcpp::Node {
public:
  MapToWorldNode() : Node("map_to_world_node") {
    std::string stl_path = "/home/ju/map.stl";
    std::string output_path = "/home/ju/turtlebotsim/src/turtlebot3/models/worlds/auto_generated_world.sdf";

    if (!std::filesystem::exists(stl_path)) {
      RCLCPP_ERROR(this->get_logger(), "STL 파일이 존재하지 않습니다: %s", stl_path.c_str());
      return;
    }

    std::ofstream sdf_file(output_path);
    if (!sdf_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "SDF 파일을 생성할 수 없습니다: %s", output_path.c_str());
      return;
    }

    sdf_file << R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100.0</mu>
                <mu2>100.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="sim_map_mesh">
    <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="map_link">
        <visual name="map_visual">
          <geometry>
            <mesh>
              <uri>file:///home/ju/map.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="map_collision">
          <geometry>
            <mesh>
              <uri>file:///home/ju/map.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
    )";

    sdf_file.close();
    RCLCPP_INFO(this->get_logger(), "SDF 파일 생성 완료: %s", output_path.c_str());
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToWorldNode>());
  rclcpp::shutdown();
  return 0;
}
