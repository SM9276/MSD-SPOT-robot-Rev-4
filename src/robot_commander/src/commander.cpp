#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.h>


class Commander : public rclcpp::Node
{
public:
    explicit Commander(const rclcpp::NodeOptions & options)
        : Node("commander", options)
    {
    }

    ~Commander()
    {
        shutdown_.store(true);
        worker_cv_.notify_all();

        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
    }


    void initialize()
    {
        // ---------------------------------------------------------
        // Parameters
        // ---------------------------------------------------------

        planning_group_ =
            get_or_declare<std::string>(
                "planning_group",
                "arm"
            );

        max_step_ =
            get_or_declare<double>(
                "max_step",
                0.005
            );

        max_pending_ =
            get_or_declare<double>(
                "max_pending",
                0.030
            );

        velocity_scaling_ =
            get_or_declare<double>(
                "velocity_scaling",
                0.05
            );

        acceleration_scaling_ =
            get_or_declare<double>(
                "acceleration_scaling",
                0.05
            );

        planning_time_ =
            get_or_declare<double>(
                "planning_time",
                1.0
            );

        use_pilz_ =
            get_or_declare<bool>(
                "use_pilz",
                true
            );

        fallback_pipeline_ =
            get_or_declare<std::string>(
                "fallback_pipeline",
                "ompl"
            );

        fallback_planner_ =
            get_or_declare<std::string>(
                "fallback_planner",
                "RRTConnectkConfigDefault"
            );


        // ---------------------------------------------------------
        // MoveGroupInterface
        // ---------------------------------------------------------

        RCLCPP_INFO(
            get_logger(),
            "Creating MoveGroupInterface..."
        );

        move_group_ =
            std::make_unique<
                moveit::planning_interface::MoveGroupInterface
            >(
                shared_from_this(),
                planning_group_
            );

        move_group_->setMaxVelocityScalingFactor(
            velocity_scaling_
        );

        move_group_->setMaxAccelerationScalingFactor(
            acceleration_scaling_
        );

        move_group_->setPlanningTime(
            planning_time_
        );

        planning_frame_ =
            move_group_->getPlanningFrame();

        end_effector_link_ =
            move_group_->getEndEffectorLink();


        // ---------------------------------------------------------
        // Subscribers
        // ---------------------------------------------------------

        relative_sub_ =
            create_subscription<geometry_msgs::msg::Vector3>(
                "/spotarm/relative_move",
                20,
                std::bind(
                    &Commander::relative_callback,
                    this,
                    std::placeholders::_1
                )
            );

        target_position_sub_ =
            create_subscription<
                geometry_msgs::msg::PointStamped
            >(
                "/spotarm/target_position",
                10,
                std::bind(
                    &Commander::position_callback,
                    this,
                    std::placeholders::_1
                )
            );

        target_pose_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped
            >(
                "/spotarm/target_pose",
                10,
                std::bind(
                    &Commander::pose_callback,
                    this,
                    std::placeholders::_1
                )
            );

        named_target_sub_ =
            create_subscription<std_msgs::msg::String>(
                "/spotarm/named_target",
                10,
                std::bind(
                    &Commander::named_target_callback,
                    this,
                    std::placeholders::_1
                )
            );

        stop_sub_ =
            create_subscription<std_msgs::msg::Empty>(
                "/spotarm/stop",
                10,
                std::bind(
                    &Commander::stop_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // ---------------------------------------------------------
        // Worker
        //
        // ROS callbacks never plan or execute.
        // They only update pending commands.
        // ---------------------------------------------------------

        worker_thread_ =
            std::thread(
                &Commander::worker_loop,
                this
            );


        // ---------------------------------------------------------
        // Information
        // ---------------------------------------------------------

        RCLCPP_INFO(
            get_logger(),
            "Commander initialized."
        );

        RCLCPP_INFO(
            get_logger(),
            "Planning group: %s",
            planning_group_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Planning frame: %s",
            planning_frame_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "End effector: %s",
            end_effector_link_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Maximum Cartesian step: %.4f m",
            max_step_
        );

        RCLCPP_INFO(
            get_logger(),
            "Maximum queued Cartesian displacement: %.4f m",
            max_pending_
        );

        RCLCPP_INFO(
            get_logger(),
            "Pilz LIN enabled: %s",
            use_pilz_ ? "true" : "false"
        );

        RCLCPP_INFO(
            get_logger(),
            "Listening on:"
        );

        RCLCPP_INFO(
            get_logger(),
            "  /spotarm/target_position"
        );

        RCLCPP_INFO(
            get_logger(),
            "  /spotarm/target_pose"
        );

        RCLCPP_INFO(
            get_logger(),
            "  /spotarm/relative_move"
        );

        RCLCPP_INFO(
            get_logger(),
            "  /spotarm/named_target"
        );

        RCLCPP_INFO(
            get_logger(),
            "  /spotarm/stop"
        );

        RCLCPP_INFO(
            get_logger(),
            "Commander ready."
        );
    }


private:

    // =============================================================
    // Parameter helper
    // =============================================================

    template<typename T>
    T get_or_declare(
        const std::string & name,
        const T & default_value)
    {
        if (!has_parameter(name))
        {
            declare_parameter<T>(
                name,
                default_value
            );
        }

        return get_parameter(name).get_value<T>();
    }


    // =============================================================
    // Relative Cartesian command
    // =============================================================

    void relative_callback(
        const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_
        );

        // An explicit absolute target is replaced by joystick jog.
        pending_pose_.reset();
        pending_position_.reset();
        pending_named_target_.reset();

        pending_relative_.x =
            std::clamp(
                pending_relative_.x + msg->x,
                -max_pending_,
                max_pending_
            );

        pending_relative_.y =
            std::clamp(
                pending_relative_.y + msg->y,
                -max_pending_,
                max_pending_
            );

        pending_relative_.z =
            std::clamp(
                pending_relative_.z + msg->z,
                -max_pending_,
                max_pending_
            );

        worker_cv_.notify_one();
    }


    // =============================================================
    // Absolute position target
    // =============================================================

    void position_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (
            !msg->header.frame_id.empty() &&
            msg->header.frame_id != planning_frame_
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "target_position frame '%s' does not match planning frame '%s'.",
                msg->header.frame_id.c_str(),
                planning_frame_.c_str()
            );

            return;
        }

        {
            std::lock_guard<std::mutex> lock(
                command_mutex_
            );

            pending_relative_ =
                geometry_msgs::msg::Vector3();

            pending_pose_.reset();

            pending_named_target_.reset();

            pending_position_ = *msg;
        }

        worker_cv_.notify_one();
    }


    // =============================================================
    // Absolute pose target
    // =============================================================

    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (
            !msg->header.frame_id.empty() &&
            msg->header.frame_id != planning_frame_
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "target_pose frame '%s' does not match planning frame '%s'.",
                msg->header.frame_id.c_str(),
                planning_frame_.c_str()
            );

            return;
        }

        {
            std::lock_guard<std::mutex> lock(
                command_mutex_
            );

            pending_relative_ =
                geometry_msgs::msg::Vector3();

            pending_position_.reset();

            pending_named_target_.reset();

            pending_pose_ = *msg;
        }

        worker_cv_.notify_one();
    }


    // =============================================================
    // Named MoveIt pose
    // =============================================================

    void named_target_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(
                command_mutex_
            );

            pending_relative_ =
                geometry_msgs::msg::Vector3();

            pending_pose_.reset();
            pending_position_.reset();

            pending_named_target_ =
                msg->data;
        }

        RCLCPP_INFO(
            get_logger(),
            "Queued named target: %s",
            msg->data.c_str()
        );

        worker_cv_.notify_one();
    }


    // =============================================================
    // Stop
    // =============================================================

    void stop_callback(
        const std_msgs::msg::Empty::SharedPtr)
    {
        RCLCPP_WARN(
            get_logger(),
            "STOP requested. Clearing pending commands."
        );

        {
            std::lock_guard<std::mutex> lock(
                command_mutex_
            );

            clear_pending_locked();
        }

        stop_requested_.store(true);

        /*
         * Ask MoveIt to cancel currently executing motion.
         *
         * This is useful for the gamepad deadman release,
         * but it is NOT a replacement for a physical E-stop.
         */
        if (move_group_)
        {
            move_group_->stop();
        }

        worker_cv_.notify_all();
    }


    // =============================================================
    // Worker
    // =============================================================

    void worker_loop()
    {
        using namespace std::chrono_literals;

        while (
            rclcpp::ok() &&
            !shutdown_.load()
        )
        {
            Command command;

            {
                std::unique_lock<std::mutex> lock(
                    command_mutex_
                );

                worker_cv_.wait_for(
                    lock,
                    100ms,
                    [this]()
                    {
                        return
                            shutdown_.load() ||
                            has_pending_command_locked();
                    }
                );

                if (shutdown_.load())
                {
                    break;
                }

                if (!has_pending_command_locked())
                {
                    continue;
                }

                command =
                    get_next_command_locked();
            }


            stop_requested_.store(false);


            bool success = false;

            switch (command.type)
            {
                case CommandType::RELATIVE:
                    success =
                        execute_relative(
                            command.relative
                        );
                    break;

                case CommandType::POSITION:
                    success =
                        execute_position(
                            command.position
                        );
                    break;

                case CommandType::POSE:
                    success =
                        execute_pose(
                            command.pose
                        );
                    break;

                case CommandType::NAMED:
                    success =
                        execute_named_target(
                            command.named_target
                        );
                    break;

                default:
                    break;
            }


            if (!success)
            {
                std::lock_guard<std::mutex> lock(
                    command_mutex_
                );

                clear_pending_locked();

                RCLCPP_WARN(
                    get_logger(),
                    "Motion failed. Pending motion queue cleared."
                );
            }
        }
    }


    // =============================================================
    // Execute relative Cartesian increment
    // =============================================================

    bool execute_relative(
        const geometry_msgs::msg::Vector3 & delta)
    {
        const auto current =
            move_group_->getCurrentPose(
                end_effector_link_
            );

        geometry_msgs::msg::Pose target =
            current.pose;

        target.position.x += delta.x;
        target.position.y += delta.y;
        target.position.z += delta.z;

        RCLCPP_INFO(
            get_logger(),
            "Cartesian step: dx=%+.4f dy=%+.4f dz=%+.4f",
            delta.x,
            delta.y,
            delta.z
        );

        RCLCPP_INFO(
            get_logger(),
            "Target: x=%.4f y=%.4f z=%.4f",
            target.position.x,
            target.position.y,
            target.position.z
        );

        return plan_and_execute_pose(
            target
        );
    }


    // =============================================================
    // Execute absolute position
    // =============================================================

    bool execute_position(
        const geometry_msgs::msg::PointStamped & target_position)
    {
        const auto current =
            move_group_->getCurrentPose(
                end_effector_link_
            );

        geometry_msgs::msg::Pose target =
            current.pose;

        target.position =
            target_position.point;

        RCLCPP_INFO(
            get_logger(),
            "Absolute position target: x=%.4f y=%.4f z=%.4f",
            target.position.x,
            target.position.y,
            target.position.z
        );

        return plan_and_execute_pose(
            target
        );
    }


    // =============================================================
    // Execute absolute pose
    // =============================================================

    bool execute_pose(
        const geometry_msgs::msg::PoseStamped & msg)
    {
        RCLCPP_INFO(
            get_logger(),
            "Absolute pose target: x=%.4f y=%.4f z=%.4f",
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        );

        return plan_and_execute_pose(
            msg.pose
        );
    }


    // =============================================================
    // Named state
    // =============================================================

    bool execute_named_target(
        const std::string & name)
    {
        RCLCPP_INFO(
            get_logger(),
            "Planning to named target '%s'...",
            name.c_str()
        );

        move_group_->stop();

        move_group_->clearPoseTargets();

        move_group_->setStartStateToCurrentState();

        move_group_->setPlanningPipelineId(
            fallback_pipeline_
        );

        if (!fallback_planner_.empty())
        {
            move_group_->setPlannerId(
                fallback_planner_
            );
        }

        if (!move_group_->setNamedTarget(name))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unknown named target '%s'.",
                name.c_str()
            );

            return false;
        }

        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        const auto result =
            move_group_->plan(plan);

        if (
            result !=
            moveit::core::MoveItErrorCode::SUCCESS
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "Planning to named target '%s' failed.",
                name.c_str()
            );

            return false;
        }

        if (stop_requested_.load())
        {
            RCLCPP_WARN(
                get_logger(),
                "Motion canceled before execution."
            );

            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Executing named target '%s'...",
            name.c_str()
        );

        const auto execution_result =
            move_group_->execute(plan);

        if (
            execution_result !=
            moveit::core::MoveItErrorCode::SUCCESS
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "Execution of named target '%s' failed.",
                name.c_str()
            );

            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Named target '%s' reached.",
            name.c_str()
        );

        return true;
    }


    // =============================================================
    // Plan a Cartesian pose
    // =============================================================

    bool plan_and_execute_pose(
        const geometry_msgs::msg::Pose & target)
    {
        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        bool planned = false;


        // ---------------------------------------------------------
        // First choice: Pilz LIN
        // ---------------------------------------------------------

        if (use_pilz_)
        {
            move_group_->clearPoseTargets();

            move_group_->setStartStateToCurrentState();

            move_group_->setPlanningPipelineId(
                "pilz_industrial_motion_planner"
            );

            move_group_->setPlannerId(
                "LIN"
            );

            move_group_->setPoseTarget(
                target,
                end_effector_link_
            );

            RCLCPP_INFO(
                get_logger(),
                "Planning Cartesian step using Pilz LIN..."
            );

            const auto result =
                move_group_->plan(plan);

            planned =
                result ==
                moveit::core::MoveItErrorCode::SUCCESS;

            if (!planned)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Pilz LIN failed. Trying OMPL fallback."
                );
            }
        }


        // ---------------------------------------------------------
        // Fallback: OMPL
        // ---------------------------------------------------------

        if (!planned)
        {
            move_group_->clearPoseTargets();

            move_group_->setStartStateToCurrentState();

            move_group_->setPlanningPipelineId(
                fallback_pipeline_
            );

            if (!fallback_planner_.empty())
            {
                move_group_->setPlannerId(
                    fallback_planner_
                );
            }

            move_group_->setPoseTarget(
                target,
                end_effector_link_
            );

            const auto result =
                move_group_->plan(plan);

            planned =
                result ==
                moveit::core::MoveItErrorCode::SUCCESS;
        }


        move_group_->clearPoseTargets();


        if (!planned)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unable to plan Cartesian movement."
            );

            return false;
        }


        if (stop_requested_.load())
        {
            RCLCPP_WARN(
                get_logger(),
                "Movement canceled before execution."
            );

            return false;
        }


        // ---------------------------------------------------------
        // Execute
        // ---------------------------------------------------------

        RCLCPP_INFO(
            get_logger(),
            "Executing trajectory..."
        );

        const auto execution_result =
            move_group_->execute(plan);

        if (
            execution_result !=
            moveit::core::MoveItErrorCode::SUCCESS
        )
        {
            RCLCPP_ERROR(
                get_logger(),
                "Trajectory execution failed."
            );

            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Trajectory completed."
        );

        return true;
    }


    // =============================================================
    // Pending command management
    // =============================================================

    enum class CommandType
    {
        NONE,
        RELATIVE,
        POSITION,
        POSE,
        NAMED
    };


    struct Command
    {
        CommandType type =
            CommandType::NONE;

        geometry_msgs::msg::Vector3 relative;

        geometry_msgs::msg::PointStamped position;

        geometry_msgs::msg::PoseStamped pose;

        std::string named_target;
    };


    bool has_pending_relative_locked() const
    {
        constexpr double epsilon =
            1e-6;

        return
            std::abs(pending_relative_.x) > epsilon ||
            std::abs(pending_relative_.y) > epsilon ||
            std::abs(pending_relative_.z) > epsilon;
    }


    bool has_pending_command_locked() const
    {
        return
            pending_named_target_.has_value() ||
            pending_pose_.has_value() ||
            pending_position_.has_value() ||
            has_pending_relative_locked();
    }


    Command get_next_command_locked()
    {
        Command command;


        // Named target has highest priority.
        if (pending_named_target_)
        {
            command.type =
                CommandType::NAMED;

            command.named_target =
                *pending_named_target_;

            pending_named_target_.reset();

            return command;
        }


        // Full pose.
        if (pending_pose_)
        {
            command.type =
                CommandType::POSE;

            command.pose =
                *pending_pose_;

            pending_pose_.reset();

            return command;
        }


        // Position-only.
        if (pending_position_)
        {
            command.type =
                CommandType::POSITION;

            command.position =
                *pending_position_;

            pending_position_.reset();

            return command;
        }


        // Relative joystick motion.
        if (has_pending_relative_locked())
        {
            command.type =
                CommandType::RELATIVE;

            const double length =
                std::sqrt(
                    pending_relative_.x *
                    pending_relative_.x +

                    pending_relative_.y *
                    pending_relative_.y +

                    pending_relative_.z *
                    pending_relative_.z
                );

            double scale =
                1.0;

            if (
                length > max_step_ &&
                length > 0.0
            )
            {
                scale =
                    max_step_ /
                    length;
            }

            command.relative.x =
                pending_relative_.x *
                scale;

            command.relative.y =
                pending_relative_.y *
                scale;

            command.relative.z =
                pending_relative_.z *
                scale;

            pending_relative_.x -=
                command.relative.x;

            pending_relative_.y -=
                command.relative.y;

            pending_relative_.z -=
                command.relative.z;

            return command;
        }


        return command;
    }


    void clear_pending_locked()
    {
        pending_relative_ =
            geometry_msgs::msg::Vector3();

        pending_position_.reset();
        pending_pose_.reset();
        pending_named_target_.reset();
    }


    // =============================================================
    // ROS
    // =============================================================

    rclcpp::Subscription<
        geometry_msgs::msg::Vector3
    >::SharedPtr relative_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped
    >::SharedPtr target_position_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr target_pose_sub_;

    rclcpp::Subscription<
        std_msgs::msg::String
    >::SharedPtr named_target_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Empty
    >::SharedPtr stop_sub_;


    // =============================================================
    // MoveIt
    // =============================================================

    std::unique_ptr<
        moveit::planning_interface::MoveGroupInterface
    > move_group_;

    std::string planning_group_;
    std::string planning_frame_;
    std::string end_effector_link_;

    std::string fallback_pipeline_;
    std::string fallback_planner_;

    double max_step_ = 0.005;
    double max_pending_ = 0.030;

    double velocity_scaling_ = 0.05;
    double acceleration_scaling_ = 0.05;

    double planning_time_ = 1.0;

    bool use_pilz_ = true;


    // =============================================================
    // Commands
    // =============================================================

    std::mutex command_mutex_;
    std::condition_variable worker_cv_;

    geometry_msgs::msg::Vector3
        pending_relative_;

    std::optional<
        geometry_msgs::msg::PointStamped
    > pending_position_;

    std::optional<
        geometry_msgs::msg::PoseStamped
    > pending_pose_;

    std::optional<std::string>
        pending_named_target_;


    // =============================================================
    // Worker
    // =============================================================

    std::thread worker_thread_;

    std::atomic<bool>
        shutdown_{false};

    std::atomic<bool>
        stop_requested_{false};
};


// =================================================================
// main
// =================================================================

int main(
    int argc,
    char ** argv)
{
    rclcpp::init(
        argc,
        argv
    );

    rclcpp::NodeOptions options;

    options
        .automatically_declare_parameters_from_overrides(
            true
        );

    auto node =
        std::make_shared<Commander>(
            options
        );

    node->initialize();

    rclcpp::executors::
        MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
