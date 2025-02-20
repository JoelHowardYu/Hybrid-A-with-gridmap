/**
 * @file simple_demo_node.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <cmath>

using namespace grid_map;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_demo_node");
    auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 1);

    // Create grid map
    GridMap map({"elevation", "traversability"});
    map.setFrameId("map");
    map.setGeometry(Length(50, 50), 1);
    RCLCPP_INFO(node->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
                map.getLength().x(), map.getLength().y(),
                map.getSize()(0), map.getSize()(1));

    // 给定一些多边形的顶点，然后遍历多边形把多边形内的高度升起来就表示障碍物
    Polygon polygon1({{-8, 0}, {-8, 5}, {-3, 5}, {-3, 0}});
    Polygon polygon2({{13, 12}, {6, 12}, {10, 4}});
    Polygon polygon3({{1, -10}, {1, -5}, {6, -5}, {6, -10}});
    Polygon polygon4({{0, -20}, {0, -25}, {5, -25}, {5, -20}});
    Polygon polygon5({{-11, 16}, {-15, 9}, {-22, 12}, {-21, 20}, {-14, 23}});
    Polygon polygon6({{-13, -4}, {-8, -12}, {-13, -20}, {-21, -5}});
    Polygon polygon7({{2, 16}, {-4, 20}, {-4, 10}});

    // 泊车
    Polygon polygon8({{25, 20}, {25, 19}, {5, 19}, {5, 20}});
    Polygon polygon9({{25, 17}, {25, 16}, {5, 16}, {5, 17}});

    Polygon polygon10({{22, 2}, {17, 5}, {12, 2}, {17, -1}});

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {

        // rclcpp::Time time = node->now();
        for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 0;
        }
        for (PolygonIterator it(map, polygon1); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon2); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon3); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon4); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon5); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon6); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon7); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon8); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon9); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        for (PolygonIterator it(map, polygon10); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }

        for (CircleIterator it(map, {16, -13}, 5); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }
        // map.setTimestamp(time.nanoseconds());
        auto msg = GridMapRosConverter::toMessage(map);
        map_pub->publish(*msg);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}