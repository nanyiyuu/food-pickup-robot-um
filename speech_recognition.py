#!/usr/bin/env python

import rospy
import speech_recognition as sr
from gtts import gTTS
from playsound import playsound
import os
from std_msgs.msg import String
import threading

class SpeechProcessingNode:
    def __init__(self):
        rospy.init_node('speech_processing_node')
        
        # 获取参数或使用默认值
        self.lang = rospy.get_param('~language', 'en')
        self.tts_output_path = rospy.get_param('~tts_output_path', '/tmp/tts_output.mp3')
        self.mpg_player = rospy.get_param('~mpg_player', 'mpg123')  # 如果继续使用 mpg123
        
        self.pub = rospy.Publisher('speech_to_text', String, queue_size=10)
        rospy.Subscriber('text_to_speech', String, self.tts_callback)
        rospy.loginfo("Speech Processing Node Initialized")
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Start speech recognition in a separate thread
        self.listener_thread = threading.Thread(target=self.listen_to_speech)
        self.listener_thread.daemon = True  # 确保线程在主线程退出时退出
        self.listener_thread.start()

    def listen_to_speech(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Listening for speech...")
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source)
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)

                # Recognize speech
                text = self.recognizer.recognize_google(audio, language=self.lang)
                rospy.loginfo(f"Recognized text: {text}")
                self.pub.publish(text)

            except sr.WaitTimeoutError:
                rospy.logwarn("Listening timed out while waiting for phrase to start")
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand the audio.")
            except sr.RequestError as e:
                rospy.logerr(f"Speech Recognition service error: {e}")
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

    def tts_callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Converting text to speech: {text}")
        self.text_to_speech(text)

    def text_to_speech(self, text):
        try:
            tts = gTTS(text=text, lang=self.lang)
            tts.save(self.tts_output_path)
            rospy.loginfo("Playing the TTS audio.")
            playsound(self.tts_output_path)
            rospy.loginfo("Text-to-Speech completed.")
        except Exception as e:
            rospy.logerr(f"Failed to convert text to speech: {e}")

if __name__ == "__main__":
    try:
        node = SpeechProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
