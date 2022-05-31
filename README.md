# ros_display_emotions
ROS package that displays emotions through faces. Working in Kinetic and Melodic.

Please install the following dependencies before running the package:

sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

The full list of emotions accepted is:

anger, annoyance, rage, anticipation, interest, vigilance, disgust, boredom, loathing, fear, apprehension, terror, joy, serenity, ecstasy, sadness, grief, pensiveness, surprise, distraction, amazement, trust, acceptance, admiration, idle

To display an emotion, first run the launch file:

roslaunch display_emotions display_emotions.launch

And then publish a message of type String to the topic /emotion:

rostopic pub /emotion std_msgs/String "data: 'joy'"
