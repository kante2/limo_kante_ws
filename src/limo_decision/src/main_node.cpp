// main_node.cpp  (LANE + LABACORN 전용 버전)

#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

// -------------------- 미션 함수 선언 --------------------
// mission_lane.cpp
void mission_lane_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_lane_step();

// mission_labacorn.cpp
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_labacorn_step();

// -------------------- 미션 상태 enum --------------------
enum MissionState
{
  MISSION_LANE = 0,
  MISSION_LABACORN
};

MissionState g_current_state = MISSION_LANE;

// -------------------- LANE / LABACORN 감지 관련 전역 --------------------

// 차선 유효 여부 (lane_perception 노드에서 /lane/valid 퍼블리시한다고 가정)
bool g_lane_valid = false;
double g_lane_camera_speed = 0.15;
double g_lane_steer_cmd = 0.0;
std::string g_lane_v2x = "D";
ros::Publisher g_lane_cmd_pub;
ros::Subscriber g_lane_steer_sub;
ros::Subscriber g_lane_v2x_sub;

// 라바콘 감지 (gap-nav perception에서 /labacorn/valid 퍼블리시한다고 가정)
bool      g_labacorn_detected        = false;
ros::Time g_labacorn_on_start_time;  // 연속 on 시작 시각
ros::Time g_labacorn_last_seen;      // 마지막으로 감지된 시각
double    g_labacorn_target_yaw      = 0.0;
bool      g_labacorn_target_valid    = false;
double    g_labacorn_follow_speed    = 1100.0;
double    g_labacorn_min_speed       = 900.0;
double    g_labacorn_k_yaw           = 1.2;
double    g_labacorn_servo_center    = 0.545;
double    g_labacorn_left_scale      = 25.0;
double    g_labacorn_right_scale     = 20.0;
bool      g_labacorn_keep_forward_on_loss = true;
ros::Publisher g_labacorn_speed_pub;
ros::Publisher g_labacorn_servo_pub;
ros::Subscriber g_labacorn_target_sub;

// 라바콘 감지 안정화 히스테리시스
const double k_labacorn_confirm_sec  = 0.5;  // 이 시간 연속 감지 시 LABACORN 진입
const double k_labacorn_release_sec  = 0.5;  // 이 시간 이상 미감지 시 LABACORN 탈출

bool g_labacorn_confirmed = false;   // 히스테리시스 거친 안정화 플래그
int  g_labacorn_count     = 0;       // LABACORN 상태 진입 횟수 카운트

// -------------------- 콜백: 차선 유효 여부 --------------------
void CB_LaneValid(const std_msgs::Bool::ConstPtr& msg)
{
  g_lane_valid = msg->data;
}

// -------------------- 콜백: 라바콘 감지 --------------------
void CB_LabacornDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_labacorn_detected = msg->data;
  g_labacorn_target_valid = msg->data;
  if (msg->data)
  {
    g_labacorn_last_seen = ros::Time::now();
  }
}

void CB_LaneSteer(const std_msgs::Float32::ConstPtr& msg)
{
  g_lane_steer_cmd = msg->data;
}

void CB_LaneV2X(const std_msgs::String::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  g_lane_v2x = msg->data;
}

void CB_LabacornTarget(const std_msgs::Float32::ConstPtr& msg)
{
  g_labacorn_target_yaw = msg->data;
}

double clamp(double value, double low, double high)
{
  if (value < low)
    return low;
  if (value > high)
    return high;
  return value;
}

double YawToServo(double yaw)
{
  const double yaw_deg = yaw * 180.0 / M_PI;
  double steering = g_labacorn_servo_center;

  if (yaw_deg < 0.0)
  {
    steering = g_labacorn_servo_center +
               (yaw_deg / g_labacorn_left_scale) * g_labacorn_servo_center;
  }
  else
  {
    steering = g_labacorn_servo_center +
               (yaw_deg / g_labacorn_right_scale) * (1.0 - g_labacorn_servo_center);
  }

  return clamp(steering, 0.0, 1.0);
}

void PublishLabacornSpeedServo(double speed, double servo)
{
  std_msgs::Float64 speed_msg;
  speed_msg.data = speed;
  std_msgs::Float64 servo_msg;
  servo_msg.data = servo;
  g_labacorn_speed_pub.publish(speed_msg);
  g_labacorn_servo_pub.publish(servo_msg);
}

// -------------------- mission_lane 구현 --------------------
void mission_lane_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  std::string lane_cmd_topic;
  std::string lane_steer_topic;
  std::string lane_v2x_topic;

  pnh.param<std::string>("lane_cmd_topic",
                         lane_cmd_topic,
                         std::string("/cmd_vel"));
  pnh.param<std::string>("lane_steer_topic",
                         lane_steer_topic,
                         std::string("/lane/steer"));
  pnh.param<std::string>("lane_v2x_topic",
                         lane_v2x_topic,
                         std::string("/path_"));
  pnh.param("lane_camera_speed", g_lane_camera_speed, 0.15);

  g_lane_cmd_pub = nh.advertise<geometry_msgs::Twist>(lane_cmd_topic, 1);
  g_lane_steer_sub = nh.subscribe(lane_steer_topic, 1, CB_LaneSteer);
  g_lane_v2x_sub = nh.subscribe(lane_v2x_topic, 1, CB_LaneV2X);

  ROS_INFO("[mission_lane] init done (cmd_topic=%s, speed=%.2f m/s)",
           lane_cmd_topic.c_str(),
           g_lane_camera_speed);
}

void mission_lane_step()
{
  geometry_msgs::Twist cmd;

  if (g_lane_valid)
  {
    cmd.linear.x = g_lane_camera_speed;
    cmd.angular.z = g_lane_steer_cmd;

    if (g_lane_v2x == "STOP")
    {
      cmd.linear.x = 0.0;
    }
  }
  else
  {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }

  g_lane_cmd_pub.publish(cmd);
}

// -------------------- mission_labacorn 구현 --------------------
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  std::string target_topic;
  std::string speed_topic;
  std::string servo_topic;

  pnh.param<std::string>("labacorn_target_topic",
                         target_topic,
                         std::string("/labacorn/target"));
  pnh.param<std::string>("labacorn_speed_topic",
                         speed_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("labacorn_servo_topic",
                         servo_topic,
                         std::string("/commands/servo/position"));

  pnh.param("labacorn_follow_speed", g_labacorn_follow_speed, 1100.0);
  pnh.param("labacorn_min_speed", g_labacorn_min_speed, 900.0);
  pnh.param("labacorn_k_yaw", g_labacorn_k_yaw, 1.2);
  pnh.param("labacorn_servo_center", g_labacorn_servo_center, 0.545);
  pnh.param("labacorn_left_scale", g_labacorn_left_scale, 25.0);
  pnh.param("labacorn_right_scale", g_labacorn_right_scale, 20.0);
  pnh.param("labacorn_keep_forward_on_loss",
            g_labacorn_keep_forward_on_loss,
            true);

  g_labacorn_speed_pub = nh.advertise<std_msgs::Float64>(speed_topic, 1);
  g_labacorn_servo_pub = nh.advertise<std_msgs::Float64>(servo_topic, 1);
  g_labacorn_target_sub =
      nh.subscribe(target_topic, 1, CB_LabacornTarget);

  ROS_INFO("[mission_labacorn] init done (speed_topic=%s, servo_topic=%s)",
           speed_topic.c_str(),
           servo_topic.c_str());
}

void mission_labacorn_step()
{
  if (!g_labacorn_target_valid)
  {
    const double speed =
        g_labacorn_keep_forward_on_loss ? g_labacorn_min_speed : 0.0;
    PublishLabacornSpeedServo(speed, g_labacorn_servo_center);
    ROS_INFO_THROTTLE(
        1.0,
        "[mission_labacorn] no target -> %s (speed=%.0f)",
        g_labacorn_keep_forward_on_loss ? "slow forward" : "stop",
        speed);
    return;
  }

  const double yaw_cmd = g_labacorn_target_yaw * g_labacorn_k_yaw;
  const double steering = YawToServo(yaw_cmd);
  PublishLabacornSpeedServo(g_labacorn_follow_speed, steering);

  ROS_INFO_THROTTLE(1.0,
                    "[mission_labacorn] speed=%.0f steer=%.3f (yaw=%.3f rad)",
                    g_labacorn_follow_speed,
                    steering,
                    g_labacorn_target_yaw);
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "autorace_main_decision");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[main_node] autorace_decision main_node started (LANE + LABACORN only)");

  // ===== 감지 토픽 이름 파라미터 =====
  std::string topic_lane_valid;
  std::string topic_labacorn_detected;

  // 차선 인식 노드(limo_lane_perception 등)에서 퍼블리시하는 토픽
  pnh.param<std::string>("lane_valid_topic", topic_lane_valid, std::string("/lane/valid"));

  // 라바콘 인식 노드(limo_labacorn_perception 등)에서 퍼블리시하는 토픽
  // 예) Bool: True = 라바콘/갭 유효 (여기로 진입 필요)
  pnh.param<std::string>("labacorn_detected_topic", topic_labacorn_detected, std::string("/labacorn/valid"));

  // ===== 감지 토픽 구독 =====
  ros::Subscriber sub_lane_valid =
      nh.subscribe(topic_lane_valid, 1, CB_LaneValid);

  ros::Subscriber sub_labacorn =
      nh.subscribe(topic_labacorn_detected, 1, CB_LabacornDetected);

  // ===== 각 미션 초기화 (lane / labacorn) =====
  mission_lane_init(nh, pnh);
  mission_labacorn_init(nh, pnh);

  ROS_INFO("[main_node] lane + labacorn mission init done");

  ros::Rate rate(10.0);  // 10 Hz
  MissionState prev_state = g_current_state;

  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();

    // -----------------------------
    // 1) 라바콘 감지 히스테리시스 (연속 on/off 시간 요구)
    // -----------------------------
    if (g_labacorn_detected)
    {
      // 처음 감지되기 시작한 시각 저장
      if (g_labacorn_on_start_time.isZero())
      {
        g_labacorn_on_start_time = now;
      }

      double on_elapsed = (now - g_labacorn_on_start_time).toSec();
      if (on_elapsed >= k_labacorn_confirm_sec)
      {
        g_labacorn_confirmed = true;
      }

      g_labacorn_last_seen = now;
    }
    else
    {
      if (!g_labacorn_last_seen.isZero())
      {
        double off_elapsed = (now - g_labacorn_last_seen).toSec();
        if (off_elapsed >= k_labacorn_release_sec)
        {
          g_labacorn_confirmed   = false;
          g_labacorn_on_start_time = ros::Time(0);
        }
      }
      else
      {
        g_labacorn_confirmed = false;
      }
    }

    // 디버깅: 차선/라바콘 상태 (1초 주기)
    ROS_INFO_THROTTLE(1.0,
                      "[main_node] lane_valid=%d | labacorn_detected=%d confirmed=%d",
                      static_cast<int>(g_lane_valid),
                      static_cast<int>(g_labacorn_detected),
                      static_cast<int>(g_labacorn_confirmed));

    // -----------------------------
    // 2) 미션 상태 결정 로직
    //    우선순위: LABACORN(안정화) > LANE
    //    (LANE은 기본 모드, lane_valid는 mission_lane 내부에서 사용 가능)
    // -----------------------------
    if (g_labacorn_confirmed)
    {
      g_current_state = MISSION_LABACORN;
    }
    else
    {
      g_current_state = MISSION_LANE;
    }

    // 상태 변경 시 로그
    if (g_current_state != prev_state)
    {
      const char* state_name = "LANE";
      if (g_current_state == MISSION_LABACORN)
        state_name = "LABACORN";

      ROS_INFO("[main_node] Mission changed -> %s", state_name);

      if (prev_state == MISSION_LABACORN &&
          g_current_state != MISSION_LABACORN)
      {
        ROS_WARN("[main_node] LABACORN exit -> %s (labacorn_detected=%d)",
                 state_name,
                 static_cast<int>(g_labacorn_detected));
      }

      if (g_current_state == MISSION_LABACORN)
      {
        ++g_labacorn_count;
        ROS_INFO("[main_node] LABACORN entry #%d (confirmed detect)",
                 g_labacorn_count);
      }

      prev_state = g_current_state;
    }

    // -----------------------------
    // 3) 현재 상태에 맞는 미션 한 스텝 실행
    // -----------------------------
    switch (g_current_state)
    {
      case MISSION_LANE:
        ROS_DEBUG("MISSION_LANE mode");
        mission_lane_step();       // 여기 안에서 /lane/steer, /lane/valid 사용해서 /cmd_vel 제어
        break;

      case MISSION_LABACORN:
        ROS_DEBUG("MISSION_LABACORN mode");
        mission_labacorn_step();   // 여기 안에서 /labacorn/yaw, /labacorn/valid 사용해서 /cmd_vel 제어
        break;

      default:
        ROS_DEBUG("MISSION_LANE (default)");
        mission_lane_step();
        break;
    }

    rate.sleep();
  }

  return 0;
}
