# display_emotions
ROS package that displays emotions through faces based on Plutchik's Wheel of Emotions [^1]. Working in Kinetic and Melodic.

[^1]: R. PLUTCHIK, R.; KELLERMAN, H. (Ed.). Emotion: theory, research and experience. New York: Academic press, 1986.

Please install the following dependencies before running the package:

    sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

Then install the package in your catkin workspace:

```bash
# Clone repository
cd catkin_ws/src/
git clone https://github.com/UtBotsAtHome-UTFPR/display_emotions.git

# Compile workspace
cd ..
catkin_make
```

## List of emotions

An **idle** face is displayed by default if no other emotion is requested. Also, eight basic emotions are possible, alongside two Low Intensity variations and one High Intensity variation for each. All of them can be requested to be displayed, even the idle face.

|*Basic Emotion*|*Low Intensity*|*High Intensity*|
| -------- | -------- | -------- |
|**joy**|**serenity** and **much_serenity**|**ecstasy**|
|**trust**|**acceptance** and **much_acceptance**|**admiration**|
|**fear**|**apprehension** and **much_apprehension**|**terror**|
|**surprise**|**distraction** and **much_distraction**|**amazement**|
|**sadness**|**pensiveness** and **much_pensiveness**|**grief**|
|**disgust**|**boredom** and **much_boredom**|**loathing**|
|**anger**|**annoyance** and **much_annoyance**|**rage**|
|**anticipation**|**interest** and **much_interest**|**vigilance**|

![image](https://user-images.githubusercontent.com/78488285/210292061-99611369-8767-46bd-8c3e-35e19a0f1197.png)[^1]

## Running

### To display an emotion, first run the launch file:

    roslaunch display_emotions display_emotions.launch

### And then publish a message of type String to the topic /emotion:

    rostopic pub /utbots/display_emotions/emotion std_msgs/String "data: 'joy'"

## Parameters:

- **faces_cycle**: make the images cycle between the most intense emotion desired or stay still at the emotion set (default: true)

- **faces_cycle_delay**: the delay in seconds to shift between emotions (only active if faces_cycle is active)
