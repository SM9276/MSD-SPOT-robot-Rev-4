#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <algorithm>
#include <cctype>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;


class Commander{
public:
    Commander(std::shared_ptr<rclcpp::Node> node){
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(0.1);
        arm_->setMaxAccelerationScalingFactor(0.1);
    }

    void goToNamedTarget(const std::string &name){
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints){
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPositionTarget(double x, double y, double z){
        arm_->setStartStateToCurrentState();
        arm_->setGoalPositionTolerance(0.01);
        arm_->setGoalOrientationTolerance(2 * M_PI);
        arm_->setPositionTarget(x, y, z, "fake_gripper");
        planAndExecute(arm_);
    }

    void goToRelativeTarget(double dx, double dy, double dz){
        arm_->setStartStateToCurrentState();
        auto current_pose = arm_->getCurrentPose("fake_gripper");
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;
        arm_->setGoalPositionTolerance(0.01);
        arm_->setGoalOrientationTolerance(2 * M_PI);
        arm_->setPoseTarget(target_pose, "fake_gripper");
        planAndExecute(arm_);
    }

    void goToPositionTargetCartesian(double x, double y, double z){
        arm_->setStartStateToCurrentState();
        auto target_pose = arm_->getCurrentPose("fake_gripper");
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        constexpr double eef_step = 0.01;
        constexpr double jump_threshold = 0.0;
        double fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);

        if (fraction < 0.999){
            RCLCPP_WARN(node_->get_logger(), "Cartesian path achieved only %.1f%% of the requested trajectory", fraction * 100.0);
            return;
        }

        arm_->execute(trajectory);
    }

    std::size_t jointCount() const{
        return arm_->getVariableCount();
    }

private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface){
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            interface->execute(plan);
        }
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    Commander commander(node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto print_help = [&commander]() {
        std::cout << "Available commands:\n";
        std::cout << "  help                         - Show this message\n";
        std::cout << "  named <target>               - Move to a named target\n";
        std::cout << "  joint <v1> ... <vN>          - Move to joint values (expected count: " << commander.jointCount() << ")\n";
        std::cout << "  position <x> <y> <z> [mode]  - Move to position; mode optional: cartesian\n";
        std::cout << "  relative <dx> <dy> <dz>       - Move relative to current position\n";
        std::cout << "  exit                         - Shutdown commander\n";
    };

    std::cout << "Commander ready. Type 'help' for command list.\n";

    std::string line;
    while (rclcpp::ok()){
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)){
            break;
        }
        if (line.empty()){
            continue;
        }

        std::istringstream iss(line);
        std::string command;
        iss >> command;

        for (char &c : command){
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }

        if (command == "exit" || command == "quit"){
            break;
        } else if (command == "help"){
            print_help();
        } else if (command == "named"){
            std::string target;
            if (!(iss >> target)){
                std::cout << "Usage: named <target>\n";
                continue;
            }
            commander.goToNamedTarget(target);
        } else if (command == "joint"){
            std::vector<double> joints;
            double value = 0.0;
            while (iss >> value){
                joints.push_back(value);
            }

            if (joints.size() != commander.jointCount()){
                std::cout << "Expected " << commander.jointCount() << " joint values, received " << joints.size() << ".\n";
                continue;
            }

            commander.goToJointTarget(joints);
        } else if (command == "position"){
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            if (!(iss >> x >> y >> z)){
                std::cout << "Usage: position <x> <y> <z> [mode]\n";
                continue;
            }
            std::string mode;
            bool cartesian = false;
            if (iss >> mode){
                for (char &c : mode){
                    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                }
                cartesian = (mode == "cartesian" || mode == "cart");
            }
            if (cartesian){
                commander.goToPositionTargetCartesian(x, y, z);
            }
            else {
                commander.goToPositionTarget(x, y, z);
            }
        } else if (command == "relative"){
            double dx = 0.0;
            double dy = 0.0;
            double dz = 0.0;
            if (!(iss >> dx >> dy >> dz)){
                std::cout << "Usage: relative <dx> <dy> <dz>\n";
                continue;
            }
            commander.goToRelativeTarget(dx, dy, dz);
        } else {
            std::cout << "Unknown command. Type 'help' for available commands.\n";
        }
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();
    return 0;
}
