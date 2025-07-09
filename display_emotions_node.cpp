#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <chrono>

class DisplayEmotionsNode : public rclcpp::Node {
public:
  DisplayEmotionsNode();

private:
  void callback(const std_msgs::msg::String::SharedPtr msg);
  void load_faces();
  void face_change();
  void publish_loop();

  image_transport::Publisher pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Image ros_image[9][6];
  cv_bridge::CvImage cv_image[9][6];
  std::string emotion_name;

  bool Param_faces_cycle;
  double Param_faces_cycle_delay;
  std::string Param_speech_gender;
  bool Param_reset_to_idle;

  int frame_direction = 0;
  int current_emotion_degree = 1;
  int desired_emotion_class = 8;
  int current_emotion_class = 8;
  int desired_emotion_degree = 5;
  int time_ = 0;
};

DisplayEmotionsNode::DisplayEmotionsNode() : Node("display_emotions_node") {
  this->declare_parameter("faces_cycle", true);
  this->declare_parameter("faces_cycle_delay", 0.25);
  this->declare_parameter("reset_to_idle", false);

  this->get_parameter("faces_cycle", Param_faces_cycle);
  this->get_parameter("faces_cycle_delay", Param_faces_cycle_delay);
  this->get_parameter("reset_to_idle", Param_reset_to_idle);

  if (Param_faces_cycle_delay <= 0)
    Param_faces_cycle_delay = 0.3;

  image_transport::ImageTransport it(this->shared_from_this());
  pub_ = it.advertise("/utbots/display_emotions/image", 1);

  sub_ = this->create_subscription<std_msgs::msg::String>(
      "/utbots/display_emotions/emotion", 10,
      std::bind(&DisplayEmotionsNode::callback, this, std::placeholders::_1));

  load_faces();

  auto delay = std::chrono::duration<double>(1.0 / (Param_faces_cycle ? Param_faces_cycle_delay : 30.0));
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(delay),
      std::bind(&DisplayEmotionsNode::publish_loop, this));
}

void DisplayEmotionsNode::face_change() {
  if (current_emotion_degree == 1) {
    current_emotion_class = desired_emotion_class;
    if (frame_direction == 0 && Param_reset_to_idle)
      desired_emotion_class = 8;
  }

  if (frame_direction == 0 && current_emotion_degree <= desired_emotion_degree)
    current_emotion_degree++;
  else
    current_emotion_degree--;

  if (current_emotion_degree >= 5 || current_emotion_degree == desired_emotion_degree)
    frame_direction = 1;

  if (current_emotion_degree <= 1)
    frame_direction = 0;
}

void DisplayEmotionsNode::load_faces() {
  std::string folder = ament_index_cpp::get_package_share_directory("display_emotions") + "/cropped/4_3/";

  for (int i = 0; i < 9; i++) {
    for (int j = 1; j < 6; j++) {
      std::string path = folder + std::to_string(i) + std::to_string(j) + ".png";
      cv_image[i][j].image = cv::imread(path, cv::IMREAD_COLOR);
      cv_image[i][j].encoding = "bgr8";
      cv_image[i][j].toImageMsg(ros_image[i][j]);
    }
  }
}

void DisplayEmotionsNode::callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_WARN(this->get_logger(), "Emotion changed to [%s]", msg->data.c_str());
  emotion_name = msg->data;

  struct EmotionMap {
    const char *name;
    int cls, deg;
  } map[] = {
      {"annoyance", 0, 2}, {"much_annoyance", 0, 3}, {"anger", 0, 4}, {"rage", 0, 5},
      {"interest", 1, 2}, {"much_interest", 1, 3}, {"anticipation", 1, 4}, {"vigilance", 1, 5},
      {"boredom", 2, 2}, {"much_boredom", 2, 3}, {"disgust", 2, 4}, {"loathing", 2, 5},
      {"apprehension", 3, 2}, {"much_apprehension", 3, 3}, {"fear", 3, 4}, {"terror", 3, 5},
      {"serenity", 4, 2}, {"much_serenity", 4, 3}, {"joy", 4, 4}, {"ecstasy", 4, 5},
      {"pensiveness", 5, 2}, {"much_pensiveness", 5, 3}, {"sadness", 5, 4}, {"grief", 5, 5},
      {"distraction", 6, 2}, {"much_distraction", 6, 3}, {"surprise", 6, 4}, {"amazement", 6, 5},
      {"acceptance", 7, 2}, {"much_acceptance", 7, 3}, {"trust", 7, 4}, {"admiration", 7, 5},
      {"idle", 8, 5},
  };

  for (const auto &e : map) {
    if (emotion_name == e.name) {
      desired_emotion_class = e.cls;
      desired_emotion_degree = e.deg;
      return;
    }
  }

  if (emotion_name == "help") {
    RCLCPP_INFO(this->get_logger(), "The full list of emotions is: annoyance, much_annoyance, anger, rage, interest, much_interest, anticipation, vigilance, boredom, much_boredom, disgust, loathing, apprehension, much_apprehension, fear, terror, serenity, much_serenity, joy, ecstasy, pensiveness, much_pensiveness, sadness, grief, distraction, much_distraction, surprise, amazement, acceptance, much_acceptance, trust, admiration, idle.");
  }
}

void DisplayEmotionsNode::publish_loop() {
  pub_.publish(ros_image[current_emotion_class][current_emotion_degree]);
  if (current_emotion_degree == 1 || current_emotion_degree == desired_emotion_degree) {
    time_++;
    if (time_ > static_cast<int>(15 * Param_faces_cycle_delay)) {
      if (!Param_faces_cycle) {
        current_emotion_class = desired_emotion_class;
        current_emotion_degree = desired_emotion_degree;
      } else {
        face_change();
      }
      time_ = 0;
    }
  } else {
    if (!Param_faces_cycle) {
      current_emotion_class = desired_emotion_class;
      current_emotion_degree = desired_emotion_degree;
    } else {
      face_change();
    }
    time_ = 0;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplayEmotionsNode>());
  rclcpp::shutdown();
  return 0;
}
