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
char path[300];
char name[3], ext[5];
sensor_msgs::Image ros_image[9][6];
cv_bridge::CvImage cv_image[9][6]; 
std::string emotion_name;
int frame = 1;
int direction = 0;
int emotion_number = 8; 
int actual_emotion = 8;
int frame_max = 5;
bool Param_faces_cycle;
double Param_faces_cycle_delay;
std::string Param_speech_gender;

// rotation between the emotions in the same category
void face_change()
{

   if (frame == 1) {
        actual_emotion = emotion_number;
   }

    if (direction==0 && frame <= frame_max)   
        frame++;
    else
        frame--;

    if (frame>=5 || frame == frame_max)
      direction = 1;

    if (frame<=1)
      direction = 0;

}

// loading the images to working memory
void load_faces()
{
  
  for(int i=0;i<9;i++)
  {
    for(int j=1;j<6;j++)
    {
      
      std::string folder = ros::package::getPath("display_emotions");
      strcpy(path,folder.c_str());  
      strcat(path, "/src/images/"); 

      name[0] = (char)i+48; 
      name[1] = (char)j+48;
      name[2] = 0;
      strcat(path,name);
    
      strcpy(ext,".png");
      ext[4] = 0;
      strcat(path,ext);

      cv_image[i][j].image = cv::imread(path,CV_LOAD_IMAGE_COLOR);
      cv_image[i][j].encoding = "bgr8";
      cv_image[i][j].toImageMsg(ros_image[i][j]);

    }
  }
}

// routine that dynamically changes the parameters on espeak node 
void reconfigure (char *param, int value)
{

  char reconfigure_msg[100], node[100], svalue[4];
	strcpy(node, "rosrun dynamic_reconfigure dynparam set /espeak_node ");
	strcpy(reconfigure_msg,node);
	strcat(reconfigure_msg, param);
	sprintf(svalue, " %d &", value);
	strcat(reconfigure_msg,svalue);
  system(reconfigure_msg);

}

// callback to read emotion request
void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_WARN("Emotion changed to [%s]", msg->data.c_str());
    emotion_name = msg->data;

    if (emotion_name == "annoyance")
    {
		  emotion_number = 0;
		  frame_max = 2;
    }
    else if (emotion_name == "much_annoyance")
    {
		  emotion_number = 0;
		  frame_max = 3;
    }
    else if (emotion_name == "anger")
    {
		  emotion_number = 0;
		  frame_max = 4;
    }
    else if (emotion_name == "rage")
    {
		  emotion_number = 0;
		  frame_max = 5;
    }
    else if (emotion_name == "interest")
    {
      emotion_number = 1;
		  frame_max = 2;
    }
    else if (emotion_name == "much_interest")
    {
      emotion_number = 1;
		  frame_max = 3;
    }
    else if (emotion_name == "anticipation")
    {
      emotion_number = 1;
		  frame_max = 4;
    }
    else if (emotion_name == "vigilance")
    {
      emotion_number = 1;
		  frame_max = 5;
    }
    else if (emotion_name == "boredom")
    {
      emotion_number = 2;
		  frame_max = 2;
    }
    else if (emotion_name == "much_boredom")
    {
      emotion_number = 2;
		  frame_max = 3;
    }
    else if (emotion_name == "disgust")
    {
      emotion_number = 2;
		  frame_max = 4;
    }
    else if (emotion_name == "loathing")
    {
      emotion_number = 2;
		  frame_max = 5;
    }
    else if (emotion_name == "apprehension")
    {
      emotion_number = 3;
		  frame_max = 2;
    }
    else if (emotion_name == "much_apprehension")
    {
      emotion_number = 3;
		  frame_max = 3;
    }
    else if (emotion_name == "fear")
    {
      emotion_number = 3;
		  frame_max = 4;
    }
    else if (emotion_name == "terror")
    {
     emotion_number = 3;
		  frame_max = 5;
    }
    else if (emotion_name == "serenity")
    {
      emotion_number = 4;
		  frame_max = 2;
    }
    else if (emotion_name == "much_serenity")
    {
      emotion_number = 4;
		  frame_max = 3;
    }
    else if (emotion_name == "joy")
    {
      emotion_number = 4;
		  frame_max = 4;
    }
    else if (emotion_name == "ecstasy")
    {
      emotion_number = 4;
		  frame_max = 5;
    }
    else if (emotion_name == "pensiveness")
    {
      emotion_number = 5;
		  frame_max = 2;
    }
    else if (emotion_name == "much_pensiveness")
    {
      emotion_number = 5;
		  frame_max = 3;
    }
    else if (emotion_name == "sadness")
    {
      emotion_number = 5;
		  frame_max = 4;
    }
    else if (emotion_name == "grief")
    {
      emotion_number = 5;
		  frame_max = 5;
    }
    else if (emotion_name == "distraction")
    {
    	emotion_number = 6; 
		  frame_max = 2;
    }
    else if (emotion_name == "much_distraction")
    {
    	emotion_number = 6; 
		  frame_max = 3;
    }
    else if (emotion_name == "surprise")
    {
    	emotion_number = 6; 
		  frame_max = 4;
    }
    else if (emotion_name == "amazement")
    {
    	emotion_number = 6; 
		  frame_max = 5;
    }
    else if (emotion_name == "acceptance")
    {
    	emotion_number = 7; 
		  frame_max = 2;
    }
    else if (emotion_name == "much_acceptance")
    {
    	emotion_number = 7; 
		  frame_max = 3;
    }
    else if (emotion_name == "trust")
    {
    	emotion_number = 7; 
		  frame_max = 4;
    }
    else if (emotion_name == "admiration")
    {
    	emotion_number = 7; 
		  frame_max = 5;
    }
    else if (emotion_name == "idle")
    {
    	emotion_number = 8; 
		  frame_max = 5;
    }
    else if (emotion_name == "help")
	ROS_INFO("The full list of emotions is: annoyance, much_annoyance, anger, rage, interest, much_interest, anticipation, vigilance, boredom, much_boredom, disgust, loathing, apprehension, much_apprehension, fear, terror, serenity, much_serenity, joy, ecstasy, pensiveness, much_pensiveness, sadness, grief, distraction, much_distraction, surprise, amazement, acceptance, much_acceptance, trust, admiration, idle.");
  
  // switching speech charateristics
  if (Param_speech_gender == "female")
  {
    reconfigure((char *)"gender",2);
  }
  else reconfigure((char *)"gender",1);

	if (emotion_number == 0) // rage
	{
    reconfigure((char *)"rate",250);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",90);
    reconfigure((char *)"range",85);
    reconfigure((char *)"wordgap",100);
	}
  else if (emotion_number == 1) // vigilance
	{
    reconfigure((char *)"rate",300);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",80);
    reconfigure((char *)"range",50);
    reconfigure((char *)"wordgap",10);
	}
  else if (emotion_number == 2) // disgust
	{
    reconfigure((char *)"rate",90);
    reconfigure((char *)"volume",50);
    reconfigure((char *)"pitch",20);
    reconfigure((char *)"range",70);
    reconfigure((char *)"wordgap",10);
	}
  else if (emotion_number == 3) // fear
	{
    reconfigure((char *)"rate",300);
    reconfigure((char *)"volume",100);
    reconfigure((char *)"pitch",90);
    reconfigure((char *)"range",85);
    reconfigure((char *)"wordgap",200);
	}
  else if (emotion_number == 4) // joy
	{
    reconfigure((char *)"rate",200);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",70);
    reconfigure((char *)"range",85);
    reconfigure((char *)"wordgap",10);
	}
  else if (emotion_number == 5) // sadness
	{
    reconfigure((char *)"rate",150);
    reconfigure((char *)"volume",50);
    reconfigure((char *)"pitch",40);
    reconfigure((char *)"range",35);
    reconfigure((char *)"wordgap",0);
	}
  else if (emotion_number == 6) // surprise
	{
    reconfigure((char *)"rate",300);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",70);
    reconfigure((char *)"range",50);
    reconfigure((char *)"wordgap",10);
	}
  else if (emotion_number == 7) // trust
	{
    reconfigure((char *)"rate",175);
    reconfigure((char *)"volume",100);
    reconfigure((char *)"pitch",50);
    reconfigure((char *)"range",50);
    reconfigure((char *)"wordgap",10);
	}
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "display_emotions_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("/emotion", 1000, callback);
  image_transport::Publisher pub = it.advertise("/face_emotion", 1);

  // parameters set in launch file
  nh.param("/display_emotions_node/faces_cycle", Param_faces_cycle, true);
  nh.param("/display_emotions_node/faces_cycle_delay", Param_faces_cycle_delay, 0.3);
  if (Param_faces_cycle_delay <= 0) Param_faces_cycle_delay = 0.3;
  nh.param<std::string>("/display_emotions_node/speech_gender", Param_speech_gender, "male");

  ros::Rate loop_rate(1/Param_faces_cycle_delay);

  load_faces();
  int time = 0;

  // if you want to see the image, uncomment the line below
  system("rosrun image_view image_view image:=/face_emotion &");

  while (ros::ok()) 
  {

    pub.publish(ros_image[actual_emotion][frame]);
    if (frame == 1 || frame == frame_max)
    {
      time ++;
      if (time > ((int)(15*Param_faces_cycle_delay)))
      {
        if (!Param_faces_cycle)
        {
          actual_emotion = emotion_number;
          frame = frame_max;
        } 
        else face_change();
        pub.publish(ros_image[actual_emotion][frame]);
      }
    }
    else
    {
      if (!Param_faces_cycle)
        {
          actual_emotion = emotion_number;
          frame = frame_max;
        } 
        else face_change();
      pub.publish(ros_image[actual_emotion][frame]);
      time = 0;
      
    }

   	ros::spinOnce();
   	loop_rate.sleep();
  }
  return 0;
}
