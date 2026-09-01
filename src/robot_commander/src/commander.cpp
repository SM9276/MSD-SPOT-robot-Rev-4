#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <atomic>

#include <unistd.h>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;


class Commander
{
public:
    explicit Commander(
        const std::shared_ptr<rclcpp::Node> &node)
        : node_(node)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Creating MoveGroupInterface..."
        );

        arm_ =
            std::make_shared<MoveGroupInterface>(
                node_,
                "arm"
            );

        arm_->setPoseReferenceFrame("base_link");
        arm_->setEndEffectorLink("fake_gripper");

        arm_->setMaxVelocityScalingFactor(0.1);
        arm_->setMaxAccelerationScalingFactor(0.1);

        arm_->setPlanningTime(10.0);
        arm_->setNumPlanningAttempts(10);

        arm_->setGoalPositionTolerance(0.01);
        arm_->setGoalOrientationTolerance(0.05);

        /*
         * XYZ position command.
         *
         * Example:
         *
         * ros2 topic pub --once \
         * /spotarm/target_position \
         * geometry_msgs/msg/PointStamped \
         * "{header: {frame_id: 'base_link'},
         *   point: {x: 0.07, y: -0.96, z: 0.109}}"
         */
        target_position_sub_ =
            node_->create_subscription<
                geometry_msgs::msg::PointStamped>(
                "/spotarm/target_position",
                10,
                [this](
                    const geometry_msgs::msg::PointStamped::
                        SharedPtr msg)
                {
                    const double x = msg->point.x;
                    const double y = msg->point.y;
                    const double z = msg->point.z;

                    enqueue(
                        [this, x, y, z]()
                        {
                            goToPositionTarget(
                                x,
                                y,
                                z
                            );
                        }
                    );
                }
            );

        /*
         * Full Cartesian pose.
         *
         * XYZ + quaternion.
         */
        target_pose_sub_ =
            node_->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/spotarm/target_pose",
                10,
                [this](
                    const geometry_msgs::msg::PoseStamped::
                        SharedPtr msg)
                {
                    geometry_msgs::msg::PoseStamped target =
                        *msg;

                    enqueue(
                        [this, target]()
                        {
                            goToPoseTarget(target);
                        }
                    );
                }
            );

        /*
         * Relative Cartesian movement.
         *
         * x = dx
         * y = dy
         * z = dz
         */
        relative_move_sub_ =
            node_->create_subscription<
                geometry_msgs::msg::Vector3>(
                "/spotarm/relative_move",
                10,
                [this](
                    const geometry_msgs::msg::Vector3::
                        SharedPtr msg)
                {
                    const double dx = msg->x;
                    const double dy = msg->y;
                    const double dz = msg->z;

                    enqueue(
                        [this, dx, dy, dz]()
                        {
                            goToRelativeTarget(
                                dx,
                                dy,
                                dz
                            );
                        }
                    );
                }
            );

        running_ = true;

        worker_thread_ =
            std::thread(
                [this]()
                {
                    workerLoop();
                }
            );

        RCLCPP_INFO(
            node_->get_logger(),
            "Commander initialized."
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning group: arm"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning frame: %s",
            arm_->getPlanningFrame().c_str()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "End effector: %s",
            arm_->getEndEffectorLink().c_str()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Listening on:"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "  /spotarm/target_position"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "  /spotarm/target_pose"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "  /spotarm/relative_move"
        );
    }


    ~Commander()
    {
        running_ = false;

        queue_condition_.notify_all();

        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
    }


    void goToNamedTarget(
        const std::string &name)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Named target: %s",
            name.c_str()
        );

        arm_->setStartStateToCurrentState();

        if (!arm_->setNamedTarget(name))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Unknown named target: %s",
                name.c_str()
            );

            return;
        }

        planAndExecute();
    }


    void goToJointTarget(
        const std::vector<double> &joints)
    {
        if (joints.size() != jointCount())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Expected %zu joints, received %zu",
                jointCount(),
                joints.size()
            );

            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning joint target..."
        );

        arm_->setStartStateToCurrentState();

        if (!arm_->setJointValueTarget(joints))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Joint target rejected."
            );

            return;
        }

        planAndExecute();
    }


    /*
     * XYZ-only target.
     *
     * Orientation is not constrained.
     */
    void goToPositionTarget(
        double x,
        double y,
        double z)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Position target:"
            " x=%.4f y=%.4f z=%.4f",
            x,
            y,
            z
        );

        arm_->setStartStateToCurrentState();

        arm_->clearPoseTargets();

        arm_->setGoalPositionTolerance(0.01);

        /*
         * Position-only target.
         */
        const bool accepted =
            arm_->setPositionTarget(
                x,
                y,
                z,
                "fake_gripper"
            );

        if (!accepted)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt rejected position target."
            );

            return;
        }

        planAndExecute();

        arm_->clearPoseTargets();
    }


    /*
     * Full Cartesian pose target.
     */
    void goToPoseTarget(
        geometry_msgs::msg::PoseStamped target)
    {
        if (target.header.frame_id.empty())
        {
            target.header.frame_id =
                "base_link";
        }

        /*
         * If quaternion is all zeros,
         * preserve the current orientation.
         */
        const double q_norm =
            std::sqrt(
                target.pose.orientation.x *
                    target.pose.orientation.x +
                target.pose.orientation.y *
                    target.pose.orientation.y +
                target.pose.orientation.z *
                    target.pose.orientation.z +
                target.pose.orientation.w *
                    target.pose.orientation.w
            );

        if (q_norm < 0.0001)
        {
            auto current =
                arm_->getCurrentPose(
                    "fake_gripper"
                );

            target.pose.orientation =
                current.pose.orientation;

            RCLCPP_WARN(
                node_->get_logger(),
                "Target quaternion was zero. "
                "Preserving current orientation."
            );
        }
        else
        {
            /*
             * Normalize quaternion.
             */
            target.pose.orientation.x /= q_norm;
            target.pose.orientation.y /= q_norm;
            target.pose.orientation.z /= q_norm;
            target.pose.orientation.w /= q_norm;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Pose target:"
            " x=%.4f y=%.4f z=%.4f",
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z
        );

        arm_->setStartStateToCurrentState();

        arm_->clearPoseTargets();

        arm_->setGoalPositionTolerance(0.01);
        arm_->setGoalOrientationTolerance(0.05);

        const bool accepted =
            arm_->setPoseTarget(
                target,
                "fake_gripper"
            );

        if (!accepted)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt rejected pose target."
            );

            return;
        }

        planAndExecute();

        arm_->clearPoseTargets();
    }


    /*
     * Relative XYZ move.
     *
     * Orientation stays unchanged.
     */
    void goToRelativeTarget(
        double dx,
        double dy,
        double dz)
    {
        arm_->setStartStateToCurrentState();

        auto current_pose =
            arm_->getCurrentPose(
                "fake_gripper"
            );

        geometry_msgs::msg::Pose target_pose =
            current_pose.pose;

        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;

        RCLCPP_INFO(
            node_->get_logger(),
            "Relative target:"
            " dx=%.4f dy=%.4f dz=%.4f",
            dx,
            dy,
            dz
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "New target:"
            " x=%.4f y=%.4f z=%.4f",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        );

        arm_->clearPoseTargets();

        arm_->setGoalPositionTolerance(0.01);
        arm_->setGoalOrientationTolerance(0.05);

        const bool accepted =
            arm_->setPoseTarget(
                target_pose,
                "fake_gripper"
            );

        if (!accepted)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt rejected relative target."
            );

            return;
        }

        planAndExecute();

        arm_->clearPoseTargets();
    }


    /*
     * Straight Cartesian path.
     *
     * Preserves current orientation.
     */
    void goToPositionTargetCartesian(
        double x,
        double y,
        double z)
    {
        arm_->setStartStateToCurrentState();

        auto current_pose =
            arm_->getCurrentPose(
                "fake_gripper"
            );

        geometry_msgs::msg::Pose target_pose =
            current_pose.pose;

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        std::vector<geometry_msgs::msg::Pose>
            waypoints;

        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory
            trajectory;

        constexpr double eef_step =
            0.005;

        constexpr double jump_threshold =
            0.0;

        RCLCPP_INFO(
            node_->get_logger(),
            "Computing Cartesian path to:"
            " x=%.4f y=%.4f z=%.4f",
            x,
            y,
            z
        );

        const double fraction =
            arm_->computeCartesianPath(
                waypoints,
                eef_step,
                jump_threshold,
                trajectory,
                true
            );

        RCLCPP_INFO(
            node_->get_logger(),
            "Cartesian path fraction: %.1f%%",
            fraction * 100.0
        );

        if (fraction < 0.999)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Cartesian path achieved only "
                "%.1f%% of requested trajectory.",
                fraction * 100.0
            );

            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Executing Cartesian trajectory..."
        );

        const auto result =
            arm_->execute(trajectory);

        if (
            result ==
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Cartesian execution succeeded."
            );
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Cartesian execution failed."
            );
        }
    }


    std::size_t jointCount() const
    {
        return arm_->getVariableCount();
    }


    /*
     * Commands from terminal are placed into
     * the same worker queue as ROS commands.
     */
    void queueNamedTarget(
        const std::string &name)
    {
        enqueue(
            [this, name]()
            {
                goToNamedTarget(name);
            }
        );
    }


    void queueJointTarget(
        const std::vector<double> &joints)
    {
        enqueue(
            [this, joints]()
            {
                goToJointTarget(joints);
            }
        );
    }


    void queuePositionTarget(
        double x,
        double y,
        double z)
    {
        enqueue(
            [this, x, y, z]()
            {
                goToPositionTarget(
                    x,
                    y,
                    z
                );
            }
        );
    }


    void queueCartesianTarget(
        double x,
        double y,
        double z)
    {
        enqueue(
            [this, x, y, z]()
            {
                goToPositionTargetCartesian(
                    x,
                    y,
                    z
                );
            }
        );
    }


    void queueRelativeTarget(
        double dx,
        double dy,
        double dz)
    {
        enqueue(
            [this, dx, dy, dz]()
            {
                goToRelativeTarget(
                    dx,
                    dy,
                    dz
                );
            }
        );
    }


private:
    /*
     * Put MoveIt operations on a separate
     * worker thread.
     *
     * This prevents ROS subscription callbacks
     * from blocking the executor while MoveIt
     * waits for actions/services.
     */
    void enqueue(
        std::function<void()> command)
    {
        {
            std::lock_guard<std::mutex>
                lock(queue_mutex_);

            command_queue_.push(
                std::move(command)
            );
        }

        queue_condition_.notify_one();
    }


    void workerLoop()
    {
        while (running_ && rclcpp::ok())
        {
            std::function<void()> command;

            {
                std::unique_lock<std::mutex>
                    lock(queue_mutex_);

                queue_condition_.wait(
                    lock,
                    [this]()
                    {
                        return
                            !command_queue_.empty() ||
                            !running_;
                    }
                );

                if (!running_)
                {
                    break;
                }

                command =
                    std::move(
                        command_queue_.front()
                    );

                command_queue_.pop();
            }

            try
            {
                command();
            }
            catch (
                const std::exception &e)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Command exception: %s",
                    e.what()
                );
            }
        }
    }


    void planAndExecute()
    {
        MoveGroupInterface::Plan plan;

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning..."
        );

        const auto result =
            arm_->plan(plan);

        if (
            result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Planning failed."
            );

            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning succeeded. Executing..."
        );

        const auto execute_result =
            arm_->execute(plan);

        if (
            execute_result ==
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Execution succeeded."
            );
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Execution failed."
            );
        }
    }


    std::shared_ptr<rclcpp::Node>
        node_;

    std::shared_ptr<MoveGroupInterface>
        arm_;


    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::
        SharedPtr target_pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped>::
        SharedPtr target_position_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::Vector3>::
        SharedPtr relative_move_sub_;


    std::mutex queue_mutex_;

    std::condition_variable
        queue_condition_;

    std::queue<
        std::function<void()>>
        command_queue_;

    std::thread worker_thread_;

    std::atomic<bool> running_{false};
};


static void printHelp(
    Commander &commander)
{
    std::cout
        << "\nAvailable commands:\n\n";

    std::cout
        << "  help\n"
        << "      Show this message\n\n";

    std::cout
        << "  named <target>\n"
        << "      Move to named target\n\n";

    std::cout
        << "  joint <v1> ... <vN>\n"
        << "      Move joints directly\n"
        << "      Expected joints: "
        << commander.jointCount()
        << "\n\n";

    std::cout
        << "  position <x> <y> <z>\n"
        << "      Move to XYZ position\n\n";

    std::cout
        << "  position <x> <y> <z> cartesian\n"
        << "      Straight Cartesian move\n\n";

    std::cout
        << "  relative <dx> <dy> <dz>\n"
        << "      Relative XYZ movement\n\n";

    std::cout
        << "  exit\n"
        << "      Shutdown commander\n\n";

    std::cout
        << "ROS topics:\n\n";

    std::cout
        << "  /spotarm/target_position\n"
        << "      geometry_msgs/msg/PointStamped\n\n";

    std::cout
        << "  /spotarm/target_pose\n"
        << "      geometry_msgs/msg/PoseStamped\n\n";

    std::cout
        << "  /spotarm/relative_move\n"
        << "      geometry_msgs/msg/Vector3\n\n";
}


int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv
    );

    auto node =
        std::make_shared<rclcpp::Node>(
            "commander",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true
                )
        );


    /*
     * MoveGroupInterface connects here.
     *
     * move_group must already be running.
     */
    Commander commander(node);


    /*
     * ROS executor runs independently of
     * the MoveIt command worker.
     */
    rclcpp::executors::
        MultiThreadedExecutor executor;

    executor.add_node(node);


    std::thread spinner(
        [&executor]()
        {
            executor.spin();
        }
    );


    RCLCPP_INFO(
        node->get_logger(),
        "Commander ready."
    );


    /*
     * If started from ros2 run in a real
     * terminal, preserve the interactive CLI.
     *
     * If launched by ros2 launch, the node
     * still works entirely through ROS topics.
     */
    if (isatty(STDIN_FILENO))
    {
        std::cout
            << "\nCommander ready. "
            << "Type 'help' for command list.\n";

        std::string line;

        while (rclcpp::ok())
        {
            std::cout
                << "> "
                << std::flush;

            if (!std::getline(
                    std::cin,
                    line))
            {
                break;
            }

            if (line.empty())
            {
                continue;
            }

            std::istringstream iss(line);

            std::string command;

            iss >> command;

            std::transform(
                command.begin(),
                command.end(),
                command.begin(),
                [](unsigned char c)
                {
                    return
                        static_cast<char>(
                            std::tolower(c)
                        );
                }
            );


            if (
                command == "exit" ||
                command == "quit")
            {
                break;
            }


            if (command == "help")
            {
                printHelp(commander);
            }


            else if (command == "named")
            {
                std::string target;

                if (!(iss >> target))
                {
                    std::cout
                        << "Usage: named <target>\n";

                    continue;
                }

                commander.queueNamedTarget(
                    target
                );
            }


            else if (command == "joint")
            {
                std::vector<double> joints;

                double value = 0.0;

                while (iss >> value)
                {
                    joints.push_back(
                        value
                    );
                }

                if (
                    joints.size() !=
                    commander.jointCount())
                {
                    std::cout
                        << "Expected "
                        << commander.jointCount()
                        << " joint values, received "
                        << joints.size()
                        << ".\n";

                    continue;
                }

                commander.queueJointTarget(
                    joints
                );
            }


            else if (command == "position")
            {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;

                if (!(iss >> x >> y >> z))
                {
                    std::cout
                        << "Usage: "
                        << "position <x> <y> <z> "
                        << "[cartesian]\n";

                    continue;
                }

                std::string mode;

                if (iss >> mode)
                {
                    std::transform(
                        mode.begin(),
                        mode.end(),
                        mode.begin(),
                        [](unsigned char c)
                        {
                            return
                                static_cast<char>(
                                    std::tolower(c)
                                );
                        }
                    );
                }

                if (
                    mode == "cartesian" ||
                    mode == "cart")
                {
                    commander.queueCartesianTarget(
                        x,
                        y,
                        z
                    );
                }
                else
                {
                    commander.queuePositionTarget(
                        x,
                        y,
                        z
                    );
                }
            }


            else if (command == "relative")
            {
                double dx = 0.0;
                double dy = 0.0;
                double dz = 0.0;

                if (!(iss >> dx >> dy >> dz))
                {
                    std::cout
                        << "Usage: "
                        << "relative "
                        << "<dx> <dy> <dz>\n";

                    continue;
                }

                commander.queueRelativeTarget(
                    dx,
                    dy,
                    dz
                );
            }


            else
            {
                std::cout
                    << "Unknown command. "
                    << "Type 'help'.\n";
            }
        }
    }
    else
    {
        /*
         * ros2 launch gives us no interactive
         * stdin. Just keep the ROS node alive.
         */
        RCLCPP_INFO(
            node->get_logger(),
            "No interactive terminal detected."
        );

        RCLCPP_INFO(
            node->get_logger(),
            "Use ROS topics to command the arm."
        );

        while (rclcpp::ok())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(250)
            );
        }
    }


    executor.cancel();

    if (spinner.joinable())
    {
        spinner.join();
    }

    rclcpp::shutdown();

    return 0;
}
