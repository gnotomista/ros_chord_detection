#!/usr/bin/env python
from pyaudio import PyAudio, paFloat32, paInt16
from time import sleep
from sys import argv
import numpy as np
import threading
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray

################################################################################
DEVICE = 0
NUM_SAMPLES = 4096
RATE = 44100
UPDATED_FREQUENCY = 10

################################################################################
class LiveAudioCapture():
    def __init__(self, device, num_samples, rate, update_frequency):
        self.pyaudio = PyAudio()
        self.device = device
        self.rate = rate
        self.num_samples = num_samples  # int(self.rate/update_frequency)
        self.record = False
        self.start_read_audio()

    def read_audio(self):
        try:
            self.audio_data = np.fromstring(self.audio_stream.read(self.num_samples), dtype=np.float32)
        except Exception as e:
            # self.record = False
            pass
        if self.record:
            self.start_thread_read_audio()
        else:
            self.audio_stream.close()
            self.pyaudio.terminate()

    def start_read_audio(self):
        self.record = True
        self.audio_data = None
        self.audio_stream = self.pyaudio.open(input_device_index=self.device, format=paFloat32, channels=1, rate=self.rate, input=True, frames_per_buffer=self.num_samples)
        self.start_thread_read_audio()

    def start_thread_read_audio(self):
        self.thread_read_audio = threading.Thread(target=self.read_audio)
        self.thread_read_audio.start()

    def close(self):
        self.record = False
        while(self.thread_read_audio.isAlive()):
            sleep(0.1)
        self.audio_stream.stop_stream()
        self.pyaudio.terminate()

################################################################################
rospy.init_node('mic_publisher', anonymous=True)
pub = rospy.Publisher('/mic_in', Float32MultiArray, queue_size=1)
audio_signal = Float32MultiArray()

live_audio_capture = LiveAudioCapture(device=DEVICE, num_samples=NUM_SAMPLES, rate=RATE, update_frequency=UPDATED_FREQUENCY)

while live_audio_capture.audio_data is None:
    pass
plt.ion()
fig = plt.figure(figsize=(20,10))
axes = fig.add_subplot(111)
axes.set_xlim(0,1023)
axes.set_ylim(-2,2)
line, = axes.plot(np.arange(0, len(live_audio_capture.audio_data), 1), live_audio_capture.audio_data, 'r-')

rate = rospy.Rate(UPDATED_FREQUENCY)
while not rospy.is_shutdown():
    audio_signal.data = live_audio_capture.audio_data
    pub.publish(audio_signal)
    rospy.loginfo("Published audio signal on /mic_in")
    line.set_ydata(live_audio_capture.audio_data)
    fig.canvas.draw()
    rate.sleep()

live_audio_capture.close()
