// main_node.cpp  (LANE + LABACORN 전용 버전)

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

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

// 라바콘 감지 (gap-nav perception에서 /labacorn/valid 퍼블리시한다고 가정)
bool      g_labacorn_detected        = false;
ros::Time g_labacorn_on_start_time;  // 연속 on 시작 시각
ros::Time g_labacorn_last_seen;      // 마지막으로 감지된 시각

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
  if (msg->data)
  {
    g_labacorn_last_seen = ros::Time::now();
  }
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
  pnh.param<std::string>("lane_valid_topic",
                         topic_lane_valid,
                         std::string("/lane/valid"));

  // 라바콘 인식 노드(limo_labacorn_perception 등)에서 퍼블리시하는 토픽
  // 예) Bool: True = 라바콘/갭 유효 (여기로 진입 필요)
  pnh.param<std::string>("labacorn_detected_topic",
                         topic_labacorn_detected,
                         std::string("/labacorn/valid"));

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
