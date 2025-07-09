#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

class CVImageView : public rclcpp::Node
{
public:
  CVImageView()
  : Node("cv_image_view")
  {
    // Declare o parâmetro com valor padrão
    this->declare_parameter<std::string>("image_topic", "/utbots/display_emotions/image");

    // Obtenha o valor do parâmetro
    std::string image_topic = this->get_parameter("image_topic").as_string();

    window_name_ = "Emotion Viewer";
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1024, 768);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10,
      std::bind(&CVImageView::image_callback, this, std::placeholders::_1));
  }

  ~CVImageView()
  {
    cv::destroyWindow(window_name_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::imshow(window_name_, cv_ptr->image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::string window_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CVImageView>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
