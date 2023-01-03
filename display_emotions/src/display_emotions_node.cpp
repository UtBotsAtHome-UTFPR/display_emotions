/*
  This package was developed by Piatan Sfair Palar and Andre Schneider de Oliveira from CPGEI - UTFPR / Curitiba - Brazil
  contact: piatan@alunos.utfpr.edu.br
*/
#define ECL_MEM_CHECK_ARRAYS

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <unistd.h>
#include "std_msgs/String.h"
#include <ros/package.h>
#include <image_transport/image_transport.h>

// global variables
sensor_msgs::Image  ros_image[9][6];
cv_bridge::CvImage  cv_image[9][6];
std::string         emotion_name;                 // the name of the emotion, read in the topic /emotion
int                 frame_direction         = 0;  // the descending (0) or ascending (1) direction of swapping emotion degree
int                 current_emotion_degree  = 1;  // the current degree of the emotion being displayed
int                 desired_emotion_class   = 8;  // the class of emotions being requested by the user
int                 current_emotion_class   = 8;  // the current emotion class. It only changes to the desired class in the first degree
int                 desired_emotion_degree  = 5;  // the desired degree of the emotion. The faces being displayed will max on this value

// ROS params
bool        Param_faces_cycle;        // when activated cycle between the emotion requested and the lesser degrees of the same emotion. If set to false, only the requested emotion is displayed
double      Param_faces_cycle_delay;  // only active if /faces_cycle is set to true. This parameter sets the delay to cycle between emotions of the same class
std::string Param_speech_gender;      // changes the gender of the speaker. It only accepts the values "male" and "female"

// rotation between the emotions in the same category
void face_change()
{
  if (current_emotion_degree == 1) // only swaps the emotion in the first image frame
    current_emotion_class = desired_emotion_class;

  if (frame_direction == 0 && current_emotion_degree <= desired_emotion_degree)
    current_emotion_degree++;
  else
    current_emotion_degree--;

  if (current_emotion_degree >= 5 || current_emotion_degree == desired_emotion_degree)
    frame_direction = 1;

  if (current_emotion_degree <= 1)
    frame_direction = 0;
}

// loading the images to working memory
void load_faces()
{
  std::string folder, ext, path;
  char name[3];

  for (int i = 0; i < 9; i++)
  {
    for (int j = 1; j < 6; j++)
    {

      folder = ros::package::getPath("display_emotions") + "/src/cropped/"; // Cropped images to Display Resolution

      // tranforming the current number on i and j to a string with that numbers
      name[0] = (char)i + 48;
      name[1] = (char)j + 48;
      name[2] = 0;

      ext = ".png";

      path = folder + name + ext;

      cv_image[i][j].image    = cv::imread(path, cv::IMREAD_COLOR);
      cv_image[i][j].encoding = "bgr8";
      cv_image[i][j].toImageMsg(ros_image[i][j]);
    }
  }
}

// callback to read emotion request
void callback(const std_msgs::String::ConstPtr &msg)
{
  ROS_WARN("Emotion changed to [%s]", msg->data.c_str());
  emotion_name = msg->data;

  if (emotion_name == "annoyance")
  {
    desired_emotion_class   = 0;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_annoyance")
  {
    desired_emotion_class   = 0;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "anger")
  {
    desired_emotion_class   = 0;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "rage")
  {
    desired_emotion_class   = 0;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "interest")
  {
    desired_emotion_class   = 1;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_interest")
  {
    desired_emotion_class   = 1;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "anticipation")
  {
    desired_emotion_class   = 1;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "vigilance")
  {
    desired_emotion_class   = 1;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "boredom")
  {
    desired_emotion_class   = 2;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_boredom")
  {
    desired_emotion_class   = 2;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "disgust")
  {
    desired_emotion_class   = 2;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "loathing")
  {
    desired_emotion_class   = 2;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "apprehension")
  {
    desired_emotion_class   = 3;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_apprehension")
  {
    desired_emotion_class   = 3;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "fear")
  {
    desired_emotion_class   = 3;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "terror")
  {
    desired_emotion_class   = 3;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "serenity")
  {
    desired_emotion_class   = 4;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_serenity")
  {
    desired_emotion_class   = 4;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "joy")
  {
    desired_emotion_class   = 4;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "ecstasy")
  {
    desired_emotion_class   = 4;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "pensiveness")
  {
    desired_emotion_class   = 5;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_pensiveness")
  {
    desired_emotion_class   = 5;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "sadness")
  {
    desired_emotion_class   = 5;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "grief")
  {
    desired_emotion_class   = 5;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "distraction")
  {
    desired_emotion_class   = 6;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_distraction")
  {
    desired_emotion_class   = 6;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "surprise")
  {
    desired_emotion_class   = 6;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "amazement")
  {
    desired_emotion_class   = 6;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "acceptance")
  {
    desired_emotion_class   = 7;
    desired_emotion_degree  = 2;
  }
  else if (emotion_name == "much_acceptance")
  {
    desired_emotion_class   = 7;
    desired_emotion_degree  = 3;
  }
  else if (emotion_name == "trust")
  {
    desired_emotion_class   = 7;
    desired_emotion_degree  = 4;
  }
  else if (emotion_name == "admiration")
  {
    desired_emotion_class   = 7;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "idle")
  {
    desired_emotion_class   = 8;
    desired_emotion_degree  = 5;
  }
  else if (emotion_name == "help")
    ROS_INFO("The full list of emotions is: annoyance, much_annoyance, anger, rage, interest, much_interest, anticipation, vigilance, boredom, much_boredom, disgust, loathing, apprehension, much_apprehension, fear, terror, serenity, much_serenity, joy, ecstasy, pensiveness, much_pensiveness, sadness, grief, distraction, much_distraction, surprise, amazement, acceptance, much_acceptance, trust, admiration, idle.");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "display_emotions_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub             = nh.subscribe("emotion", 1000, callback);
  image_transport::Publisher pub  = it.advertise("image", 1);

  // parameters set in launch file
  nh.param("/voice/emotion/display_emotions_node/faces_cycle", Param_faces_cycle, true);
  nh.param("/voice/emotion/display_emotions_node/faces_cycle_delay", Param_faces_cycle_delay, 0.3);
  nh.param<std::string>("/voice/emotion/display_emotions_node/speech_gender", Param_speech_gender, "male");

  if (Param_faces_cycle_delay <= 0)
    Param_faces_cycle_delay = 0.3;

  double rate = 1 / Param_faces_cycle_delay;
  if (Param_faces_cycle == false)
    rate = 30;
  ros::Rate loop_rate(rate);

  load_faces();
  int time = 0;

  // if you want to see the image, uncomment the line below
  // system("rosrun image_view image_view image:=/face_emotion &");

  while (ros::ok())
  {

    pub.publish(ros_image[current_emotion_class][current_emotion_degree]);
    if (current_emotion_degree == 1 || current_emotion_degree == desired_emotion_degree)
    {
      time++;
      if (time > ((int)(15 * Param_faces_cycle_delay)))
      {
        if (!Param_faces_cycle)
        {
          current_emotion_class = desired_emotion_class;
          current_emotion_degree = desired_emotion_degree;
        }
        else
          face_change();
        pub.publish(ros_image[current_emotion_class][current_emotion_degree]);
      }
    }
    else
    {
      if (!Param_faces_cycle)
      {
        current_emotion_class = desired_emotion_class;
        current_emotion_degree = desired_emotion_degree;
      }
      else
        face_change();
      pub.publish(ros_image[current_emotion_class][current_emotion_degree]);
      time = 0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
