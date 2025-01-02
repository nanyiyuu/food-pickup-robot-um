#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    To run this script, you need to run the following lines in terminal:
    - [Robot] $ roslaunch rchomeedu_vision multi_astra.launch
    - [IO] $ roslaunch astra_camera astra.launch
    - [Juno] $ roslaunch usb_cam usb_cam-test.launch
    - roslaunch opencv_apps face_detection.launch image:=/camera/rgb/image_raw
    - roslaunch kids_module say_hello.launch
    - rosrun your_package speech_to_text_ros.py
"""

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import speech_recognition as sr
import threading

class SpeechToTextROS:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('speech_to_text_ros', anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # 初始化SoundClient用于语音播放
        self.soundhandle = SoundClient()
        rospy.sleep(1)  # 等待SoundClient连接
        self.soundhandle.stopAll()
        rospy.loginfo("SpeechToTextROS Node Initialized.")

        # 创建发布者，将识别的文本发布到话题
        self.text_pub = rospy.Publisher('/recognized_text', String, queue_size=10)

        # 初始化语音识别器
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # 设置全局标志，防止多线程访问冲突
        self.lock = threading.Lock()

        # 启动语音识别线程
        self.stop_listening = self.recognizer.listen_in_background(self.microphone, self.callback)
        rospy.loginfo("Started listening to microphone...")

    def callback(self, recognizer, audio):
        """
        语音识别回调函数，当检测到音频时调用
        """
        with self.lock:
            try:
                # 使用Google Web Speech API进行识别，设置语言为中文
                text = recognizer.recognize_google(audio, language="zh-CN")
                rospy.loginfo(f"Recognized Speech: {text}")

                # 发布识别的文本
                self.text_pub.publish(text)

                # 根据识别的文本执行相应操作
                self.process_command(text)

            except sr.UnknownValueError:
                rospy.logwarn("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"Could not request results from Google Speech Recognition service; {e}")

    def process_command(self, text):
        """
        根据识别的文本执行相应操作
        """
        rospy.loginfo(f"Processing command: {text}")
        if "hello" in text:
            self.soundhandle.say("hello")
        elif "whether" in text:
            self.soundhandle.say("sorry")
        elif "thank" in text:
            self.soundhandle.say("thank")
        else:
            self.soundhandle.say("抱歉，我没有理解您的意思。")

    def cleanup(self):
        """
        清理函数，在节点关闭时调用
        """
        rospy.loginfo("Shutting down SpeechToTextROS node...")
        self.stop_listening(wait_for_stop=False)
        self.soundhandle.stopAll()

if __name__ == "__main__":
    try:
        SpeechToTextROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
