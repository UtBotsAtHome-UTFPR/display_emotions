#define ECL_MEM_CHECK_ARRAYS

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string.h>
#include <stdio.h>
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
char emotion_name[30];
int frame = 1;
int direction = 0;
int emotion_number = 8; 
int actual_emotion = 8;
int frame_max = 5;

// rotation between the emotions in the same category
void face_change(){

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
void load_faces(){
  
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

// switching speech charateristics 
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

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_WARN("Emotion changed to [%s]", msg->data.c_str());
    strcpy(emotion_name,msg->data.c_str());
                      
    if (strcmp(emotion_name,"annoyance")==0)
    {
		  emotion_number = 0;
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_annoyance")==0)
    {
		  emotion_number = 0;
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"anger")==0)
    {
		  emotion_number = 0;
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"rage")==0)
    {
		  emotion_number = 0;
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"interest")==0)
    {
      emotion_number = 1;
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_interest")==0)
    {
      emotion_number = 1;
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"anticipation")==0)
    {
      emotion_number = 1;
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"vigilance")==0)
    {
      emotion_number = 1;
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"boredom")==0)
    {
      emotion_number = 2;
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_boredom")==0)
    {
      emotion_number = 2;
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"disgust")==0)
    {
      emotion_number = 2;
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"loathing")==0)
    {
      emotion_number = 2;
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"apprehension")==0)
    {
      emotion_number = 3;
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_apprehension")==0)
    {
      emotion_number = 3;
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"fear")==0)
    {
      emotion_number = 3;
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"terror")==0)
    {
     emotion_number = 3;
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"serenity")==0)
    {
      emotion_number = 4;
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_serenity")==0)
    {
      emotion_number = 4;
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"joy")==0)
    {
      emotion_number = 4;
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"ecstasy")==0)
    {
      emotion_number = 4;
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"pensiveness")==0)
    {
      emotion_number = 5;
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_pensiveness")==0)
    {
      emotion_number = 5;
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"sadness")==0)
    {
      emotion_number = 5;
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"grief")==0)
    {
      emotion_number = 5;
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"distraction")==0)
    {
    	emotion_number = 6; 
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_distraction")==0)
    {
    	emotion_number = 6; 
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"surprise")==0)
    {
    	emotion_number = 6; 
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"amazement")==0)
    {
    	emotion_number = 6; 
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"acceptance")==0)
    {
    	emotion_number = 7; 
		  frame_max = 2;
    }
    else if (strcmp(emotion_name,"much_acceptance")==0)
    {
    	emotion_number = 7; 
		  frame_max = 3;
    }
    else if (strcmp(emotion_name,"trust")==0)
    {
    	emotion_number = 7; 
		  frame_max = 4;
    }
    else if (strcmp(emotion_name,"admiration")==0)
    {
    	emotion_number = 7; 
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"idle")==0)
    {
    	emotion_number = 8; 
		  frame_max = 5;
    }
    else if (strcmp(emotion_name,"help")==0)
	ROS_INFO("The full list of emotions is: annoyance, much_annoyance, anger, rage, interest, much_interest, anticipation, vigilance, boredom, much_boredom, disgust, loathing, apprehension, much_apprehension, fear, terror, serenity, much_serenity, joy, ecstasy, pensiveness, much_pensiveness, sadness, grief, distraction, much_distraction, surprise, amazement, acceptance, much_acceptance, trust, admiration, idle.");

	if (emotion_number == 0) // rage
	{
    reconfigure((char *)"rate",250);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",90);
    reconfigure((char *)"range",85);
    reconfigure((char *)"wordgap",100);
	}
  if (emotion_number == 1) // vigilance
	{
    reconfigure((char *)"rate",300);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",80);
    reconfigure((char *)"range",50);
    reconfigure((char *)"wordgap",10);
	}
  if (emotion_number == 2) // disgust
	{
    reconfigure((char *)"rate",90);
    reconfigure((char *)"volume",50);
    reconfigure((char *)"pitch",20);
    reconfigure((char *)"range",70);
    reconfigure((char *)"wordgap",10);
	}
  if (emotion_number == 3) // fear
	{
    reconfigure((char *)"rate",300);
    reconfigure((char *)"volume",100);
    reconfigure((char *)"pitch",90);
    reconfigure((char *)"range",85);
    reconfigure((char *)"wordgap",200);
	}
  if (emotion_number == 4) // joy
	{
    reconfigure((char *)"rate",200);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",70);
    reconfigure((char *)"range",85);
    reconfigure((char *)"wordgap",10);
	}
  if (emotion_number == 5) // sadness
	{
    reconfigure((char *)"rate",150);
    reconfigure((char *)"volume",50);
    reconfigure((char *)"pitch",40);
    reconfigure((char *)"range",35);
    reconfigure((char *)"wordgap",0);
	}
  if (emotion_number == 6) // surprise
	{
    reconfigure((char *)"rate",300);
    reconfigure((char *)"volume",150);
    reconfigure((char *)"pitch",70);
    reconfigure((char *)"range",50);
    reconfigure((char *)"wordgap",10);
	}
  if (emotion_number == 7) // trust
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
  ros::Rate loop_rate(7);

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
		if (time > 15)
		{
			face_change();
			pub.publish(ros_image[actual_emotion][frame]);
		}
	}
	else
	{
		face_change();
		pub.publish(ros_image[actual_emotion][frame]);
		time = 0;
		
	}

   	ros::spinOnce();
   	loop_rate.sleep();
  }
  return 0;
}
