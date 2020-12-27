#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "Chromagram.h"
#include "ChordDetector.h"

#define NUM_SAMPLES 4096
#define RATE 44100

std::vector<std::string> chordFundamental = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "Ab", "A", "Bb", "B"};
std::string chordString;

Chromagram* chromagram;
ChordDetector* chordDetector;
ros::Publisher chordPub;
std_msgs::Int8MultiArray chordMsg;

void micInCallback(const std_msgs::Float32MultiArray::ConstPtr& recvMsg)
{
  if (recvMsg->data.size() > 0) {
    ROS_INFO("Received audio signal: %f ...", recvMsg->data[0]);
    std::vector<double> audioSignal(recvMsg->data.begin(), recvMsg->data.end());

    chromagram->processAudioFrame(audioSignal);
    if (chromagram->isReady()) {
      std::string chordString;
      std::vector<double> chroma = chromagram->getChromagram();
      chordDetector->detectChord(chroma);

      switch (chordDetector->quality) {
        case ChordDetector::ChordQuality::Major:
          if (chordDetector->intervals == 0) {
            chordString = chordFundamental[chordDetector->rootNote];
          } else { // == 7
            chordString = chordFundamental[chordDetector->rootNote] + "maj7";//"\u03947";
          }
          break;
        case ChordDetector::ChordQuality::Minor:
          if (chordDetector->intervals == 0) {
            chordString = chordFundamental[chordDetector->rootNote] + "-";
          } else { // == 7
            chordString = chordFundamental[chordDetector->rootNote] + "-7";
          }
          break;
        case ChordDetector::ChordQuality::Dominant:
          chordString = chordFundamental[chordDetector->rootNote] + "7";
          break;
        case ChordDetector::ChordQuality::Suspended:
          if (chordDetector->intervals == 2) {
            chordString = chordFundamental[chordDetector->rootNote] + "sus2";
          } else if (chordDetector->intervals == 4) {
            chordString = chordFundamental[chordDetector->rootNote] + "sus4";
          }
          break;
        case ChordDetector::ChordQuality::Diminished5th:
          chordString = chordFundamental[chordDetector->rootNote] + "dim";//"\u26AC";
          break;
        case ChordDetector::ChordQuality::Augmented5th:
          chordString = chordFundamental[chordDetector->rootNote] + "aug";
          break;
      }
      // std::cout << "Chromagram: ";
      // for (int i = 0; i < chroma.size(); i++) {
      //   std::cout << chroma[i] << " ";
      // }
      // std::cout << std::endl;
      // std::cout << "Chord detected: " << chordString;
      // std::cout << "\n\tRoot note: " << chordDetector->rootNote
      //           << "\n\tQuality: " << chordDetector->quality
      //           << "\n\tIntervals: " << chordDetector->intervals << std::endl;

      chordMsg.data = {(signed char)chordDetector->rootNote, (signed char)chordDetector->quality, (signed char)chordDetector->intervals};
      chordPub.publish(chordMsg);
      ROS_INFO("Published chord: %s (%d %d %d)", chordString.c_str(), chordDetector->rootNote, chordDetector->quality, chordDetector->intervals);
    }
  }
}

int main(int argc, char **argv)
{
  chromagram = new Chromagram(NUM_SAMPLES, RATE);
  chromagram->setChromaCalculationInterval(NUM_SAMPLES);
  chordDetector = new ChordDetector();

  ros::init(argc, argv, "chord_publisher");
  ros::NodeHandle n;
  chordPub = n.advertise<std_msgs::Int8MultiArray>("/chords", 1);
  ros::Subscriber robotPosesSub = n.subscribe<std_msgs::Float32MultiArray>("/mic_in", 1, micInCallback);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
