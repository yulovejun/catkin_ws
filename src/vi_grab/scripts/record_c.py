#!/usr/bin/env python3
import os
import pyaudio
import wave
import time
import signal
import rospy
from std_msgs.msg import Int8

def record_audio():
    def signal_handler(signal, frame):
        nonlocal recording
        print("录音结束.")
        recording = False

    # 创建PyAudio对象
    audio = pyaudio.PyAudio()

    # 设置录音参数
    format = pyaudio.paInt16  # 采样位深度
    channels = 1  # 声道数
    rate = 44100  # 采样率
    frames_per_buffer = 1024

    # 打开音频输入流
    stream = audio.open(format=format,
                        channels=channels,
                        rate=rate,
                        input=True,
                        frames_per_buffer=frames_per_buffer)

    print("录音中... (按Ctrl+C中断录音)")

    frames = []
    recording = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while recording:
            data = stream.read(frames_per_buffer)
            frames.append(data)
    except KeyboardInterrupt:
        pass

    # 关闭音频输入流
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # 创建文件夹（如果不存在）
    folder_name = "record_c"
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)

    # 生成时间戳作为文件名
    timestamp = int(time.time())
    filename = os.path.join(folder_name, f"record_c_{timestamp}.wav")

    # 保存录音文件
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(audio.get_sample_size(format))
        wf.setframerate(rate)
        wf.writeframes(b''.join(frames))

    print(f"录音已保存为 {filename}")
    rospy.signal_shutdown("录音完成并关闭节点")

if __name__ == "__main__":
    rospy.init_node('record_demo')
    record_msg = rospy.wait_for_message("/awake_flag", Int8, timeout=None)
    print(record_msg)
    if record_msg.data == 1:
        record_audio()
    while not rospy.is_shutdown():
        # 等待 ROS 节点关闭
        rospy.sleep(1)
    
