// /**
//  * @brief Publishes a MarkerArray of points and Markers for obstacles to ROS topics.
//  *
//  * This class reads point data from a file, creates a MarkerArray of points and Markers for obstacles, and publishes them to ROS topics at a regular interval.
//  *
//  * The point data is read from a file specified by the `points.txt` filename. The indices of the x, y, and (optional) z coordinates are read from a file specified by the `coordinate_indices.txt` filename.
//  *
//  * The MarkerArray contains a Marker for each point, with the first and last points displayed as cubes and the rest as spheres. The Markers are published to the `point_matrix` topic.
//  *
//  * The Markers for obstacles are published to the `obstacles` topic. The obstacles are defined as four cubes with fixed positions and dimensions.
//  *
//  * The publishing is done on a timer that fires every 1 second.
//  */

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <fstream>
// #include <sstream>
// #include <vector>

// // It is alright if the following imports throw errors in your IDE, since they are not built by a CMake compiler, but rather by colcon.
// #include <rclcpp/rclcpp.hpp>
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/pose2_d.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// // Function to read a file and store its contents as a matrix
// std::vector<std::vector<double>> read_file_as_matrix(std::string file_name)
// {
//     std::ifstream file(file_name);
//     std::string line;
//     std::vector<std::vector<double>> matrix;
//     while (std::getline(file, line))
//     {
//         if(line.empty())    // Reached end of matrix
//             break;
//         std::istringstream iss(line);

//         if (line.empty()) {
//             break;
//         }
        
//         std::vector<double> row;
//         double value;
//         while (iss >> value)
//         {
//             row.push_back(value);
//         }
//         matrix.push_back(row);
//     }

//     std::cout << "Number of rows/points: " << matrix.size() << std::endl;
//     return matrix;
// }

// // Function to read a file and store its contents as a matrix
// std::string read_file_as_string(std::string file_name)
// {
//     std::ifstream file(file_name);
//     std::string line;
//     std::getline(file, line);
//     std::istringstream iss(line);

//     return line;
// }

// // Class to publish the trajectory and obstacles as a point matrix and obstacles as ROS markers
// class PointMatrixPublisher : public rclcpp::Node
// {
// public:
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
//     rclcpp::TimerBase::SharedPtr timer_;

//     PointMatrixPublisher(std::vector<std::vector<double>> matrix, std::vector<double> index_matrix, std::string obstacle) : Node("point_matrix_publisher")
//     {
//         // Create publishers for point matrix and obstacles
//         marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("point_matrix", 10);
//         marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", 10);

//         // Create a timer to publish points periodically
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this, matrix, index_matrix, obstacle]()
//                                          { this->publishPoints(matrix, index_matrix, obstacle); });
//     }

// private:
//     // Function to publish points and obstacles as ROS markers
//     void publishPoints(std::vector<std::vector<double>> matrix, std::vector<double> index_matrix, std::string obstacle)
//     {
//         auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

//         size_t rows = matrix.size();

//         visualization_msgs::msg::Marker marker,
//             lines;
//         marker.header.frame_id = "map"; // Change to your desired frame
//         lines.header.frame_id = "map";

//         marker.ns = "points";
//         lines.ns = "points";

//         for (size_t row = 0; row < rows; ++row)
//         {
//             marker.header.stamp = this->now();
//             lines.header.stamp = this->now();

//             marker.id = row;
//             lines.id = row + 3102;

//             // Set the position of the marker based on the matrix and index_matrix
//             marker.pose.position.x = matrix[row][index_matrix[0]];
//             marker.pose.position.y = matrix[row][index_matrix[1]];
//             marker.pose.position.z = index_matrix.size() > 2 ? matrix[row][index_matrix[2]] : 0.0; // Assuming z-coordinate is in the third column, so if there are only two columns, then z-coordinate is 0.
//             marker.action = visualization_msgs::msg::Marker::ADD;

//             // Set the marker type and color based on the row index
//             if (row == 0)
//             {
//                 marker.type = visualization_msgs::msg::Marker::CUBE;
//                 marker.color.a = 1.0;
//                 marker.color.r = 0.0;
//                 marker.color.g = 1.0;
//                 marker.color.b = 0.0;
//                 marker.scale.x = 0.15;
//                 marker.scale.y = 0.15;
//                 marker.scale.z = 0.05;
//             }
//             else if (row == rows - 1)
//             {
//                 marker.type = visualization_msgs::msg::Marker::CUBE;
//                 marker.color.a = 1.0;
//                 marker.color.r = 0.7;
//                 marker.color.g = 0.0;
//                 marker.color.b = 1.0;
//                 marker.scale.x = 0.15;
//                 marker.scale.y = 0.15;
//                 marker.scale.z = 0.05;
//             }
//             else
//             {
//                 marker.type = visualization_msgs::msg::Marker::SPHERE;
//                 if(matrix[row][matrix[0].size() - 1] != matrix[row - 1][matrix[0].size() - 1]) {
//                     marker.scale.x = 0.15;
//                     marker.scale.y = 0.15;
//                     marker.scale.z = 0.15;
//                     marker.color.a = 0.5;
//                     marker.color.r = 1.0;
//                     marker.color.g = 0.0;
//                     marker.color.b = 0.0;
//                 }
//                 else {
//                     marker.scale.x = 0.05;
//                     marker.scale.y = 0.05;
//                     marker.scale.z = 0.05;
//                     marker.color.a = 0.5;
//                     marker.color.r = 1.0;
//                     marker.color.g = 1.0;
//                     marker.color.b = 1.0;
//                 }
//             }

//             marker_array_msg->markers.push_back(marker);
//             marker_array_msg->markers.push_back(lines);
//         }

//         if(obstacle.find("y") != std::string::npos) {
//             // Define and publish obstacles as ROS markers
//             visualization_msgs::msg::Marker obs1, obs2, obs3, obs4;
//             rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("talker");
//             rclcpp::Time now = node->get_clock()->now();
//             rclcpp::Duration lifetime_time(0, 0); // 0 makes it forever

//             obs1.type = obs2.type = obs3.type = obs4.type = visualization_msgs::msg::Marker::CUBE;
//             obs1.header.frame_id = obs2.header.frame_id = obs3.header.frame_id = obs4.header.frame_id = "/map";

//             obs1.header.stamp = obs2.header.stamp = obs3.header.stamp = obs4.header.stamp = now;
//             obs1.ns = obs2.ns = obs3.ns = obs4.ns = "obstacles";
//             obs1.lifetime = obs2.lifetime = obs3.lifetime = obs4.lifetime = lifetime_time;
//             obs1.action = obs2.action = obs3.action = obs4.action = visualization_msgs::msg::Marker::ADD;

//             obs1.id = 0;
//             obs2.id = 1;
//             obs3.id = 2;
//             obs4.id = 3;

//             // Set the dimensions and positions of the obstacles
//             obs1.scale.x = obs2.scale.x = 4.5;
//             obs1.scale.y = obs2.scale.y = 0.5;
//             obs3.scale.y = 1;
//             obs3.scale.x = 0.5;
//             obs4.scale.x = 2;
//             obs4.scale.y = 0.5;

//             obs1.scale.z = obs2.scale.z = obs3.scale.z = obs4.scale.z = 0;

//             obs1.pose.position.x = obs1.scale.x / 2; // flat 2x12 rect @ (8, 6)
//             obs1.pose.position.y = 3 - obs1.scale.x / 2 + 2;
//             // obs1.pose.position.z = 0.25;
//             obs1.pose.orientation.x = 0.0;
//             obs1.pose.orientation.y = 0.0;
//             obs1.pose.orientation.z = 0.0;
//             obs1.pose.orientation.w = 1;
//             obs1.color.a = 1;
//             obs1.color.r = obs1.color.g = obs1.color.b = 255.0f;

//             obs2.pose.position.x = obs2.scale.x / 2; // flat 2x12 rect @ (14, 14)
//             obs2.pose.position.y = 1.5 - obs2.scale.x / 2 + 2;
//             // obs2.pose.position.z = 0.25;
//             obs2.pose.orientation.x = 0.0;
//             obs2.pose.orientation.y = 0.0;
//             obs2.pose.orientation.z = 0.0;
//             obs2.pose.orientation.w = 1;
//             obs2.color.a = 1;
//             obs2.color.r = obs2.color.g = obs2.color.b = 255.0f;

//             obs3.pose.position.x = obs3.scale.x / 2; // flat 7x4 rect @ (16.5, 2)
//             obs3.pose.position.y = .5 - obs3.scale.y / 2 + 2;
//             // obs3.pose.position.z = 0.25;
//             obs3.pose.orientation.x = 0.0;
//             obs3.pose.orientation.y = 0.0;
//             obs3.pose.orientation.z = 0.0;
//             obs3.pose.orientation.w = 1;
//             obs3.color.a = 1;
//             obs3.color.r = obs3.color.g = obs3.color.b = 255.0f;

//             obs4.pose.position.x = 1;
//             obs4.pose.position.y = 1;
//             obs4.pose.orientation.x = 0.0;
//             obs4.pose.orientation.y = 0.0;
//             obs4.pose.orientation.z = 0.0;
//             obs4.color.a = 1;
//             obs4.color.r = obs4.color.g = obs4.color.b = 255.0f;

//             marker_array_msg->markers.push_back(obs1);
//             marker_array_msg->markers.push_back(obs2);
//             marker_array_msg->markers.push_back(obs3);
//         }

//         marker_pub_->publish(*marker_array_msg);
//     }
// };

// std::shared_ptr<PointMatrixPublisher> node;

// void signal_callback_handler(int signum) {
//    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
//    visualization_msgs::msg::Marker marker;
//    marker.action = visualization_msgs::msg::Marker::DELETEALL;
//    marker_array_msg->markers.push_back(marker);
//    node->marker_pub_->publish(*marker_array_msg);
//    rclcpp::sleep_for(std::chrono::milliseconds(100)); // Give time for publish
//    exit(signum);
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     signal(SIGINT, signal_callback_handler);
//     // Read the coordinate indices and points from files
//     std::vector<double> index_matrix = read_file_as_matrix("coordinate_indices.txt")[0];
//     std::vector<std::vector<double>> matrix = read_file_as_matrix("src/points.txt");
//     std::string obstacle = read_file_as_string("obstacles.txt");

//     // Create and spin the PointMatrixPublisher node
//     node = std::make_shared<PointMatrixPublisher>(matrix, index_matrix, obstacle);
//     rclcpp::spin(node);
//     return 0;
// }
