# ros_chord_detection
ROS package for chord detection from audio signal captured from microphone. All code is packaged in Docker.

The chord detection algorithm used is developed in *Real-Time Chord Recognition For Live Performance, A. M. Stark and M. D. Plumbley. In Proceedings of the 2009 International Computer Music Conference (ICMC 2009), Montreal, Canada, 16-21 August 2009* and the code can be found on GitHub [here](https://github.com/adamstark/Chord-Detector-and-Chromagram).

The whole ROS-based backend used for chord detection is containerized for portability. Example scripts to interface with the backend are provided in Python and MATLAB. See below for getting the system running.

## Setup
1. Install [Docker](https://docs.docker.com/engine/install/)
2. In a terminal:
```
git clone https://github.com/gnotomista/ros_chord_detection
cd ros_chord_detection/docker
sh build.sh
sh run.sh
```
At this point, two more terminal windows will open. In the one named *mic_publisher_thread.py*, it asks to select the audio input device (e.g. microphone) from which the audio signal is to be captured:
3. Select the corresponding number and hit Enter.  
If everything is successful, chords detected by analyzing the audio signal should be published using ROS.
4. Examples of how to grab detected chords using Python and MATLAB are provided in the [examples](https://github.com/gnotomista/ros_chord_detection/tree/main/examples) folder:
* Python
  * Install [ROS Toolbox](https://www.mathworks.com/products/ros.html)
  * Open a terminal
  * `python ros_chord_detection/examples/chord_sub.py`
* MATLAB
  * Install [ROS](http://wiki.ros.org/Installation/)
  * Launch MATLAB
  * `run ros_chord_detection/examples/chord_sub.m`
