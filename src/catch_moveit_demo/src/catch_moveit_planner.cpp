#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveGroupInterface 是 MoveIt2 中最常用的高级 C++ 接口：
// 它负责和 move_group 通信，完成 IK、规划、轨迹执行等动作。
using moveit::planning_interface::MoveGroupInterface;

namespace
{

double posePositionDistance(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double poseOrientationDistance(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b)
{
  const double dot =
      a.pose.orientation.x * b.pose.orientation.x +
      a.pose.orientation.y * b.pose.orientation.y +
      a.pose.orientation.z * b.pose.orientation.z +
      a.pose.orientation.w * b.pose.orientation.w;
  const double clamped = std::min(1.0, std::max(-1.0, std::abs(dot)));
  return 2.0 * std::acos(clamped);
}

geometry_msgs::msg::PoseStamped makePoseFromParameters(const rclcpp::Node::SharedPtr& node)
{
  // 这个函数只用于 plan_on_start:=true 的单点测试模式。
  // 不依赖视觉节点，直接从 ROS 参数构造一个目标 PoseStamped。
  const auto frame_id = node->get_parameter("target_frame").as_string();
  const auto x = node->get_parameter("target_x").as_double();
  const auto y = node->get_parameter("target_y").as_double();
  const auto z = node->get_parameter("target_z").as_double();
  const auto roll = node->get_parameter("target_roll").as_double();
  const auto pitch = node->get_parameter("target_pitch").as_double();
  const auto yaw = node->get_parameter("target_yaw").as_double();

  // MoveIt 的目标姿态使用四元数。这里把更直观的 RPY 弧度转成四元数。
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = node->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

bool planAndMaybeExecute(
    const rclcpp::Logger& logger,
    MoveGroupInterface& move_group,
    const geometry_msgs::msg::PoseStamped& target,
    const std::string& end_effector_link,
    bool execute)
{
  // 清理上一次目标，避免旧目标残留影响本次规划。
  move_group.clearPoseTargets();

  // 把目标位姿设置到指定末端 link。
  // Piper MoveIt SRDF 中 arm 组默认 tip 是 link6；如果你之后建了真实 TCP link，
  // 这里的 end_effector_link 应该改成那个 TCP link。
  const bool target_ok = move_group.setPoseTarget(target, end_effector_link);
  if (!target_ok)
  {
    RCLCPP_ERROR(logger, "setPoseTarget failed for frame=%s link=%s",
                 target.header.frame_id.c_str(), end_effector_link.c_str());
    return false;
  }

  RCLCPP_INFO(
      logger,
      "Planning to pose frame=%s link=%s position=(%.4f, %.4f, %.4f) "
      "orientation=(%.4f, %.4f, %.4f, %.4f)",
      target.header.frame_id.c_str(),
      end_effector_link.c_str(),
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z,
      target.pose.orientation.x,
      target.pose.orientation.y,
      target.pose.orientation.z,
      target.pose.orientation.w);

  MoveGroupInterface::Plan plan;

  // plan() 只做规划，不会让机械臂动。
  // 它会请求 move_group 根据当前 joint_states、URDF/SRDF、碰撞模型和规划器参数生成轨迹。
  const auto result = move_group.plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "MoveIt planning failed");
    move_group.clearPoseTargets();
    return false;
  }

  const auto point_count = plan.trajectory_.joint_trajectory.points.size();
  RCLCPP_INFO(logger, "MoveIt planning succeeded, trajectory points=%zu", point_count);

  if (!execute)
  {
    // 安全默认值：只验证 MoveIt 能否规划到目标点。
    // 真正让机械臂运动必须显式设置 execute:=true。
    RCLCPP_WARN(logger, "Dry-run mode: plan was not executed. Set execute:=true to move the arm.");
    move_group.clearPoseTargets();
    return true;
  }

  // execute() 会把规划得到的轨迹发给 MoveIt 控制器执行。
  // 真实机械臂测试时请确保 piper_ros、MoveIt 控制器、急停和周围空间都准备好。
  const auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "MoveIt execution failed");
    move_group.clearPoseTargets();
    return false;
  }

  RCLCPP_INFO(logger, "MoveIt execution succeeded");
  move_group.clearPoseTargets();
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("catch_moveit_planner");
  const auto logger = node->get_logger();

  // planning_group 对应 piper_with_gripper_moveit/config/piper.srdf 里的 group name。
  // 当前机械臂 1-6 轴规划组是 arm。
  node->declare_parameter<std::string>("planning_group", "arm");

  // end_effector_link 是 MoveIt 设置 PoseTarget 时使用的末端 link。
  // 当前 SRDF 的 arm chain 是 base_link -> link6，所以默认 link6。
  node->declare_parameter<std::string>("end_effector_link", "link6");

  // 视觉节点 catch_ros 会发布顶部预抓取目标到这个话题。
  // 本节点订阅该 PoseStamped 后调用 MoveIt 规划。
  node->declare_parameter<std::string>("target_pose_topic", "/catch/top_pregrasp_pose");

  // execute=false：只规划不执行。execute=true：规划成功后执行轨迹。
  node->declare_parameter<bool>("execute", false);

  // plan_on_start=true：启动后不等视觉话题，直接用 target_x/y/z/rpy 参数规划一次。
  // 适合验证 MoveIt、控制器和机械臂是否能通。
  node->declare_parameter<bool>("plan_on_start", false);

  // MoveIt 规划参数：时间、尝试次数、速度/加速度比例和目标容差。
  node->declare_parameter<double>("planning_time", 5.0);
  node->declare_parameter<int>("planning_attempts", 5);
  node->declare_parameter<double>("max_velocity_scaling", 0.20);
  node->declare_parameter<double>("max_acceleration_scaling", 0.20);
  node->declare_parameter<double>("position_tolerance", 0.01);
  node->declare_parameter<double>("orientation_tolerance", 0.10);

  // 视觉节点可能每帧都发布一个略有抖动的目标。
  // 下面三个参数用于目标去重：短时间内不重复规划，相近目标不重复规划。
  node->declare_parameter<double>("min_plan_interval_sec", 1.0);
  node->declare_parameter<double>("target_position_delta", 0.02);
  node->declare_parameter<double>("target_orientation_delta", 0.15);

  // plan_on_start 模式使用的手动目标位姿，单位：
  // x/y/z 为米，roll/pitch/yaw 为弧度。
  node->declare_parameter<std::string>("target_frame", "base_link");
  node->declare_parameter<double>("target_x", 0.20);
  node->declare_parameter<double>("target_y", 0.00);
  node->declare_parameter<double>("target_z", 0.32);
  node->declare_parameter<double>("target_roll", 0.0);
  node->declare_parameter<double>("target_pitch", 1.5708);
  node->declare_parameter<double>("target_yaw", 0.0);

  const auto planning_group = node->get_parameter("planning_group").as_string();
  const auto end_effector_link = node->get_parameter("end_effector_link").as_string();
  const auto target_pose_topic = node->get_parameter("target_pose_topic").as_string();
  const bool execute = node->get_parameter("execute").as_bool();
  const bool plan_on_start = node->get_parameter("plan_on_start").as_bool();
  const double min_plan_interval_sec = node->get_parameter("min_plan_interval_sec").as_double();
  const double target_position_delta = node->get_parameter("target_position_delta").as_double();
  const double target_orientation_delta = node->get_parameter("target_orientation_delta").as_double();

  // MoveGroupInterface 内部需要接收 action、service、joint_states 等回调。
  // 因此这里单独开一个 executor 线程持续 spin 当前节点。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // 初始化 MoveIt 规划接口。这里会连接 move_group，并加载对应 planning group。
  MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink(end_effector_link);
  move_group.setPlanningTime(node->get_parameter("planning_time").as_double());
  move_group.setNumPlanningAttempts(node->get_parameter("planning_attempts").as_int());
  move_group.setMaxVelocityScalingFactor(node->get_parameter("max_velocity_scaling").as_double());
  move_group.setMaxAccelerationScalingFactor(node->get_parameter("max_acceleration_scaling").as_double());
  move_group.setGoalPositionTolerance(node->get_parameter("position_tolerance").as_double());
  move_group.setGoalOrientationTolerance(node->get_parameter("orientation_tolerance").as_double());

  // 目标 PoseStamped 默认使用 base_link。catch_ros 发布的 /catch/top_pregrasp_pose
  // 也应使用 base_link，这样 MoveIt 无需额外 TF 转换。
  move_group.setPoseReferenceFrame("base_link");

  RCLCPP_INFO(logger, "MoveIt planner ready: group=%s, eef=%s, topic=%s, execute=%s",
              planning_group.c_str(), end_effector_link.c_str(), target_pose_topic.c_str(),
              execute ? "true" : "false");

  std::atomic_bool planning_busy{ false };
  bool has_last_accepted_target = false;
  auto last_plan_start_time = node->now() - rclcpp::Duration::from_seconds(min_plan_interval_sec);
  geometry_msgs::msg::PoseStamped last_accepted_target;

  // 主功能：收到视觉节点发布的顶部预抓取 Pose 后，调用 MoveIt 规划。
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic, rclcpp::QoS(10),
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 规划可能耗时较长。如果上一条目标还在规划，直接丢弃新的目标，
        // 避免多个规划请求互相覆盖。
        if (planning_busy.exchange(true))
        {
          RCLCPP_WARN(logger, "Planner is busy; ignoring incoming target pose");
          return;
        }

        auto target = *msg;
        if (target.header.frame_id.empty())
        {
          // frame_id 为空时兜底为 base_link。
          target.header.frame_id = "base_link";
        }

        const auto now = node->now();
        const double seconds_since_last_plan = (now - last_plan_start_time).seconds();
        if (seconds_since_last_plan < min_plan_interval_sec)
        {
          RCLCPP_WARN_THROTTLE(
              logger, *node->get_clock(), 1000,
              "Target received too soon after previous plan; waiting %.2f s",
              min_plan_interval_sec);
          planning_busy.store(false);
          return;
        }

        if (has_last_accepted_target)
        {
          const double position_delta = posePositionDistance(target, last_accepted_target);
          const double orientation_delta = poseOrientationDistance(target, last_accepted_target);
          if (position_delta < target_position_delta && orientation_delta < target_orientation_delta)
          {
            RCLCPP_WARN_THROTTLE(
                logger, *node->get_clock(), 2000,
                "Target pose change is small; ignoring repeated target "
                "(position_delta=%.4f m, orientation_delta=%.4f rad)",
                position_delta, orientation_delta);
            planning_busy.store(false);
            return;
          }
        }

        last_plan_start_time = now;
        last_accepted_target = target;
        has_last_accepted_target = true;
        planAndMaybeExecute(logger, move_group, target, end_effector_link, execute);
        planning_busy.store(false);
      });

  (void)sub;

  if (plan_on_start)
  {
    // 手动单点测试路径：不用相机、不用 YOLO，启动后直接规划一次。
    planning_busy.store(true);
    const auto target = makePoseFromParameters(node);
    planAndMaybeExecute(logger, move_group, target, end_effector_link, execute);
    planning_busy.store(false);
  }

  // 进程保持运行，等待 /catch/top_pregrasp_pose。
  // 真正的 ROS 回调由上面的 spinner 线程处理。
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok())
  {
    rate.sleep();
  }

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
