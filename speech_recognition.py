#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import threading

class SpeechProcessingNode:
    def __init__(self):
        rospy.init_node('speech_processing_node')

        # Publisher for speech to text
        self.pub = rospy.Publisher('speech_to_text', String, queue_size=10)

        # Subscriber for text to speech
        rospy.Subscriber('text_to_speech', String, self.tts_callback)

        # Initialize sound client
        self.soundhandle = SoundClient()
        rospy.sleep(1)  # Wait for sound_play to initialize

        rospy.loginfo("Speech Processing Node Initialized")

        # Start a thread to listen to speech_to_text topic from pocketsphinx_ros
        self.listener_thread = threading.Thread(target=self.listen_to_speech)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def listen_to_speech(self):
        # Subscribe to the output of pocketsphinx_ros
        rospy.Subscriber('/pocketsphinx/output', String, self.speech_callback)

    def speech_callback(self, msg):
        rospy.loginfo(f"Recognized text: {msg.data}")
        self.pub.publish(msg.data)

    def tts_callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Converting text to speech: {text}")
        self.play_text(text)

    def play_text(self, text):
        # Use sound_play to say the text
        self.soundhandle.say(text)
        rospy.loginfo("Text-to-Speech completed.")

if __name__ == "__main__":
    try:
        node = SpeechProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
