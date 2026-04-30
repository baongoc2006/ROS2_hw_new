#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <numeric>
#include <fstream>
#include <iomanip>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// ─── Kết quả mỗi lần thử nghiệm ────────────────────────────────────────────
struct TrialResult {
  std::string planner_name;
  int         trial_number;
  double      duration_sec;
  double      distance_m;
  bool        had_recovery;
  bool        succeeded;
};

// ─── Vị trí odometry dùng để tính quãng đường ───────────────────────────────
struct OdomPos { double x, y; };

class PlannerBenchmarkNode : public rclcpp::Node
{
public:
  PlannerBenchmarkNode()
  : Node("planner_benchmark"),
    m_trial_done(false),
    m_had_recovery(false),
    m_current_distance(0.0)
  {
    m_nav_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Subscribe /odom để tích lũy quãng đường
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        OdomCallback(msg);
      });

    RCLCPP_INFO(this->get_logger(),
      "PlannerBenchmarkNode ready. Waiting for action server...");

    if (!m_nav_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting benchmark...");

    // Chạy benchmark trong thread riêng để không block executor
    m_benchmark_thread = std::thread([this]() { RunBenchmark(); });
    m_benchmark_thread.detach();
  }

private:
  // ── Tích lũy quãng đường từ odometry ─────────────────────────────────────
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    OdomPos current{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y
    };

    if (m_odom_initialized) {
      double dx = current.x - m_last_odom.x;
      double dy = current.y - m_last_odom.y;
      m_current_distance += std::sqrt(dx * dx + dy * dy);
    }

    m_last_odom       = current;
    m_odom_initialized = true;
  }

  // ── Gửi một goal và chờ kết quả ─────────────────────────────────────────
  bool RunOneTrial(double goal_x, double goal_y, double goal_yaw,
                   double & out_duration, double & out_distance)
  {
    m_trial_done      = false;
    m_trial_succeeded = false;
    m_had_recovery    = false;

    // Reset bộ đếm quãng đường
    m_current_distance = 0.0;
    m_odom_initialized = false;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x  = goal_x;
    goal_msg.pose.pose.position.y  = goal_y;
    goal_msg.pose.pose.orientation.z = std::sin(goal_yaw / 2.0);
    goal_msg.pose.pose.orientation.w = std::cos(goal_yaw / 2.0);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        // Heuristic detect recovery: nếu ETA tăng đột biến nghĩa là đang recovery
        static double last_eta = 9999.0;
        double eta = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
        if (eta > last_eta + 3.0) {
          m_had_recovery = true;
          RCLCPP_WARN(this->get_logger(), "  [!] Recovery behavior detected (ETA jumped)");
        }
        last_eta = eta;
      };

    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result)
      {
        m_trial_succeeded = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        m_trial_done      = true;
      };

    auto start = std::chrono::steady_clock::now();
    m_nav_client->async_send_goal(goal_msg, send_goal_options);

    // Chờ kết quả (timeout 120s)
    auto timeout = std::chrono::seconds(120);
    auto deadline = start + timeout;
    while (!m_trial_done && rclcpp::ok()) {
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_WARN(this->get_logger(), "Trial timeout!");
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    auto end = std::chrono::steady_clock::now();
    out_duration = std::chrono::duration<double>(end - start).count();
    out_distance = m_current_distance;

    return m_trial_succeeded;
  }

  // ── Chạy toàn bộ benchmark ───────────────────────────────────────────────
  void RunBenchmark()
  {
    // Goal cố định: A=(0,0) → B=(1.5,-0.5)
    constexpr double GOAL_X   =  1.5;
    constexpr double GOAL_Y   = -0.5;
    constexpr double GOAL_YAW =  0.0;
    constexpr int    N_TRIALS =  5;

    std::vector<std::string> planners = {"NavFn", "SmacPlanner2D"};
    std::vector<TrialResult> all_results;

    for (const auto & planner : planners) {
      RCLCPP_INFO(this->get_logger(),
        "\n════════════════════════════════════════");
      RCLCPP_INFO(this->get_logger(),
        " PLANNER: %s", planner.c_str());
      RCLCPP_INFO(this->get_logger(),
        "════════════════════════════════════════");

      for (int i = 1; i <= N_TRIALS; ++i) {
        RCLCPP_INFO(this->get_logger(),
          " Trial %d/%d ...", i, N_TRIALS);

        double duration = 0.0, distance = 0.0;
        bool ok = RunOneTrial(GOAL_X, GOAL_Y, GOAL_YAW, duration, distance);

        TrialResult r;
        r.planner_name = planner;
        r.trial_number = i;
        r.duration_sec = duration;
        r.distance_m   = distance;
        r.had_recovery = m_had_recovery;
        r.succeeded    = ok;
        all_results.push_back(r);

        RCLCPP_INFO(this->get_logger(),
          "  → %s | Time: %.2f s | Dist: %.2f m | Recovery: %s",
          ok ? "SUCCEEDED" : "FAILED",
          duration, distance,
          m_had_recovery ? "YES" : "no");

        // Về start trước trial tiếp theo
        if (i < N_TRIALS) {
          RCLCPP_INFO(this->get_logger(), "  Returning to start...");
          double d_dummy, dist_dummy;
          RunOneTrial(0.0, 0.0, 0.0, d_dummy, dist_dummy);
          rclcpp::sleep_for(std::chrono::seconds(2));
        }
      }
    }

    PrintReport(all_results);
    SaveCsv(all_results);
    rclcpp::shutdown();
  }

  // ── In bảng kết quả ra console ───────────────────────────────────────────
  void PrintReport(const std::vector<TrialResult> & results)
  {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(),
      "┌────────────────────┬───────┬─────────────┬──────────────┬───────────┬──────────┐");
    RCLCPP_INFO(this->get_logger(),
      "│ Planner            │ Trial │  Time (s)   │  Dist (m)    │ Recovery  │ Result   │");
    RCLCPP_INFO(this->get_logger(),
      "├────────────────────┼───────┼─────────────┼──────────────┼───────────┼──────────┤");

    for (const auto & r : results) {
      RCLCPP_INFO(this->get_logger(),
        "│ %-18s │   %d   │ %9.2f   │ %10.2f   │ %-9s │ %-8s │",
        r.planner_name.c_str(), r.trial_number,
        r.duration_sec, r.distance_m,
        r.had_recovery ? "YES" : "no",
        r.succeeded    ? "SUCCEEDED" : "FAILED");
    }

    RCLCPP_INFO(this->get_logger(),
      "└────────────────────┴───────┴─────────────┴──────────────┴───────────┴──────────┘");

    // Thống kê trung bình và độ lệch chuẩn cho từng planner
    for (const std::string & planner : {"NavFn", "SmacPlanner2D"}) {
      std::vector<double> times, dists;
      for (const auto & r : results) {
        if (r.planner_name == planner && r.succeeded) {
          times.push_back(r.duration_sec);
          dists.push_back(r.distance_m);
        }
      }
      if (times.empty()) continue;

      auto mean = [](const std::vector<double> & v) {
        return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
      };
      auto stddev = [&mean](const std::vector<double> & v) {
        double m = mean(v);
        double sq_sum = 0.0;
        for (double x : v) sq_sum += (x - m) * (x - m);
        return std::sqrt(sq_sum / v.size());
      };

      RCLCPP_INFO(this->get_logger(),
        "[%s] Time: mean=%.2f s  std=%.2f s | Dist: mean=%.2f m  std=%.2f m",
        planner.c_str(),
        mean(times), stddev(times),
        mean(dists),  stddev(dists));
    }
  }

  // ── Lưu kết quả ra file CSV ──────────────────────────────────────────────
  void SaveCsv(const std::vector<TrialResult> & results)
  {
    const std::string path = "/tmp/planner_benchmark_results.csv";
    std::ofstream f(path);
    f << "planner,trial,duration_sec,distance_m,had_recovery,succeeded\n";
    for (const auto & r : results) {
      f << r.planner_name << ","
        << r.trial_number << ","
        << std::fixed << std::setprecision(3) << r.duration_sec << ","
        << r.distance_m   << ","
        << (r.had_recovery ? "true" : "false") << ","
        << (r.succeeded    ? "true" : "false") << "\n";
    }
    RCLCPP_INFO(this->get_logger(), "Results saved to: %s", path.c_str());
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr   m_nav_client;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  std::thread m_benchmark_thread;

  // Trạng thái trial hiện tại
  std::atomic<bool>   m_trial_done{false};
  std::atomic<bool>   m_trial_succeeded{false};
  bool                m_had_recovery{false};
  double              m_current_distance{0.0};

  // Odometry tracking
  OdomPos m_last_odom{0.0, 0.0};
  bool    m_odom_initialized{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
