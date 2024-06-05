
# Problem Statement: TeleOperations using ROS

This project, developed as a submission for the Foundations of Design Practicum (FDP) course, utilizes ROS, Arduino (C++), and Python (primarily with Tkinter).

## How it works
The robot receives commands from a mini controller built with Tkinter. The terminal server acts as the publisher, while the NodeMCU on the robot is the subscriber. The NodeMCU subscribes to a topic where the publisher sends commands.


## Prerequesites
### Software
##### Ubuntu 20.04
##### ROS Noetic Desktop
##### rosserial_server
### Hardware
##### Nodemcu (ESP8266)
##### Motor driver 
##### Motors with wheels
##### Battery
##### Buck convertor and capacitors (for safety)



## Run Locally

Clone the project

```bash
  git clone https://github.com/AkshatJha0411/teleop
```

Start the ROS server
```bash
  roslaunch rosserial_server socket.launch
```


Go to the project directory in terminal

```bash
  python3 ui_teleop.py
```

The tkinter UI should open and then the the robot should be able to operate using commands given by user.

## Video demo
Not the most aesthetically pleasing demo, but gives an overview of robots capabilities. Will take a better video soon (hopefully)

[Video link](https://vimeo.com/949310056?share=copy)

## Team
##### [Basil Khan](https://github.com/Basilkhan1718)
##### [Dheeraj Jha](https://github.com/Dheerajjha14)
##### [Shubhankit Singh](https://github.com/shubhankitsingh)
