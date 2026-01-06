#include <chrono>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class PCDToGridMapNode : public rclcpp::Node {
public:
  PCDToGridMapNode() : Node("pcd_to_gridmap_node") {
    this->declare_parameter("pcd_filename", "");
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("max_slope_deg", 45.0); // degrees
    this->declare_parameter("min_height", -10.0);
    this->declare_parameter("max_height", 10.0);

    pcd_filename_ = this->get_parameter("pcd_filename").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();

    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "grid_map", rclcpp::QoS(1).transient_local());
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());

    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_cloud", rclcpp::QoS(1).transient_local());

    if (!pcd_filename_.empty()) {
      processPCD();
    } else {
      RCLCPP_WARN(this->get_logger(), "No PCD filename provided.");
    }
  }

private:
  void processPCD() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename_, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s",
                   pcd_filename_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points from %s", cloud->size(),
                pcd_filename_.c_str());

    // Store the loaded cloud for periodic visualization
    pcl::toROSMsg(*cloud, cloud_msg_);
    cloud_msg_.header.frame_id = map_frame_id_;

    // Create a timer to publish the cloud every 2 seconds
    timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
      if (cloud_msg_.data.size() > 0) {
        cloud_msg_.header.stamp = this->now(); // Update timestamp
        pcd_pub_->publish(cloud_msg_);
      }
    });

    // Initialize Grid Map
    grid_map::GridMap map({"elevation", "slope", "traversability"});
    map.setFrameId(map_frame_id_);
    map.setBasicLayers({"elevation"});

    // Use GridMapPclLoader for robust PCD to GridMap conversion
    grid_map::GridMapPclLoader pclLoader(this->get_logger());
    pclLoader.setInputCloud(cloud);

    // Configure parameters for the loader (could be exposed as ROS params)
    // For now using defaults or simple logic

    // We can manually populate if PclLoader is too complex for this simple
    // case, but official recommendations suggest using it. However, let's keep
    // it simple and robust for this node.

    // Find cloud bounds
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    int valid_points = 0;
    for (const auto &pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;
      if (std::abs(pt.x) > 10000.0 || std::abs(pt.y) > 10000.0 ||
          std::abs(pt.z) > 100.0)
        continue;

      min_x = std::min(min_x, (double)pt.x);
      max_x = std::max(max_x, (double)pt.x);
      min_y = std::min(min_y, (double)pt.y);
      max_y = std::max(max_y, (double)pt.y);
      valid_points++;
    }

    if (valid_points == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "No valid points found in PCD after filtering.");
      return;
    }

    // Use robot-centric fixed size or smaller bounds
    // With corrupted PCD, we should be careful.
    // Let's use a 500m x 500m area around the valid center if it looks
    // reasonable
    double length_x = std::min(max_x - min_x + 2.0, 500.0);
    double length_y = std::min(max_y - min_y + 2.0, 500.0);

    // Use the resolution parameter for the grid map
    double target_resolution = resolution_;

    grid_map::Position center((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);

    // If center is way off (0,0), use (0,0)
    if (std::abs(center.x()) > 1000.0 || std::abs(center.y()) > 1000.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Calculated center (%f, %f) is suspect. Defaulting to (0,0).",
                  center.x(), center.y());
      center = grid_map::Position(0, 0);
    }

    map.setGeometry(grid_map::Length(length_x, length_y), target_resolution,
                    center);

    RCLCPP_INFO(this->get_logger(),
                "Grid Map initialized with size %f x %f at res %f", length_x,
                length_y, target_resolution);

    // Populate elevation layer using MEAN pooling (sum and count)
    // We need a temporary map for count to calculate mean
    map.add("count", 0.0);

    for (const auto &pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;
      if (std::abs(pt.x) > 10000.0 || std::abs(pt.y) > 10000.0 ||
          std::abs(pt.z) > 100.0)
        continue;

      grid_map::Position pos(pt.x, pt.y);
      if (map.isInside(pos)) {
        grid_map::Index index;
        map.getIndex(pos, index);

        float current_elevation = map.at("elevation", index);
        if (!std::isfinite(current_elevation)) {
          map.at("elevation", index) = pt.z;
          map.at("count", index) = 1.0;
        } else {
          map.at("elevation", index) += pt.z;
          map.at("count", index) += 1.0;
        }
      }
    }

    // Normalize to get Mean Elevation
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
         ++iterator) {
      const grid_map::Index index(*iterator);
      if (map.at("count", index) > 0.0) {
        map.at("elevation", index) /= map.at("count", index);
      }
    }

    // Gap filling (Inpainting) logic
    // We iterate a few times to fill small holes by checking neighbors
    for (int i = 0; i < 3; ++i) {
      grid_map::GridMap map_copy = map;
      for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
           ++iterator) {
        const grid_map::Index index(*iterator);
        if (!std::isfinite(map.at("elevation", index))) {
          float sum = 0.0;
          int count = 0;
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              if (dx == 0 && dy == 0)
                continue;
              grid_map::Index neighborIndex = index + grid_map::Index(dx, dy);
              if (map_copy.isValid(neighborIndex)) {
                float val = map_copy.at("elevation", neighborIndex);
                if (std::isfinite(val)) {
                  sum += val;
                  count++;
                }
              }
            }
          }
          if (count > 0) {
            map.at("elevation", index) = sum / count;
          }
        }
      }
    }

    // Apply Smoothing (Box Blur) to reduce aliasing artifacts
    grid_map::GridMap map_smoothed = map;
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
         ++iterator) {
      const grid_map::Index index(*iterator);
      if (std::isfinite(map.at("elevation", index))) {
        float sum = 0.0;
        int count = 0;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            grid_map::Index neighborIndex = index + grid_map::Index(dx, dy);
            if (map.isValid(neighborIndex)) {
              float val = map.at("elevation", neighborIndex);
              if (std::isfinite(val)) {
                sum += val;
                count++;
              }
            }
          }
        }
        if (count > 0) {
          map_smoothed.at("elevation", index) = sum / count;
        }
      }
    }
    map = map_smoothed;

    // Calculate Slope and Traversability
    double max_slope_deg = this->get_parameter("max_slope_deg").as_double();
    double max_slope_tan = std::tan(max_slope_deg * M_PI / 180.0);

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
         ++iterator) {
      const grid_map::Index index(*iterator);

      // Simple slope calculation (difference with neighbors)
      // In a real app, we'd use grid_map_filters, but for a single node this is
      // fine.
      float slope = 0.0;
      float elevation = map.at("elevation", index);

      if (std::isfinite(elevation)) {
        // Check neighbors for max slope
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0)
              continue;
            grid_map::Index neighborIndex = index + grid_map::Index(dx, dy);
            if (map.isValid(neighborIndex)) {
              float neighborElevation = map.at("elevation", neighborIndex);
              if (std::isfinite(neighborElevation)) {
                float s = std::abs(elevation - neighborElevation) / resolution_;
                slope = std::max(slope, s);
              }
            }
          }
        }
        map.at("slope", index) = slope;

        if (slope > max_slope_tan) {
          map.at("traversability", index) = 0.0; // Not traversable
        } else {
          map.at("traversability", index) = 1.0; // Traversable
        }
      }
    }

    // Publish Grid Map
    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(map);
    grid_map_pub_->publish(std::move(grid_map_msg));

    // Convert to Occupancy Grid for Nav2
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "traversability", 0.0,
                                                   1.0, occupancy_grid);

    // Invert occupancy grid: 1.0 (traversable) should be 0 (free), 0.0
    // (untraversable) should be 100 (obstacle) Actually toOccupancyGrid takes
    // min/max values for data range. If min_value=0.0 and max_value=1.0, then
    // 0.0 becomes 100 and 1.0 becomes 0.

    occupancy_grid_pub_->publish(occupancy_grid);
    RCLCPP_INFO(this->get_logger(), "Published Grid Map and Occupancy Grid.");
  }

  std::string pcd_filename_;
  double resolution_;
  std::string map_frame_id_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      occupancy_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDToGridMapNode>());
  rclcpp::shutdown();
  return 0;
}
