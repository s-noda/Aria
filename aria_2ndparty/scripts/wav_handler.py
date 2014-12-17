#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import wave
import pyaudio
import re
import subprocess
import numpy


__play__ = False
__filename__ = ["\0"]
__begin__ = [0]

__amp__ = 1.0
__count__ = 0
__play_range__ = 0
__wf__ = None
__chunk__ = 1024

__p__ = None
__stream__ = None


def callback(data):
    str_get = data.data    # data.data = "filename, start, end, amp, speed"
    str_split = str_get.split(", ")
    global __play__

    if str_split[0] == "quit":    # data.data = "quit"
        if __play__:
            __play__ = False
        return

    if __play__:
       return

    global __filename__
    global __begin__
    global __wf__

    # data.data = "continue, which_speech, end, amp, speed"
    if str_split[0] == "continue":
        which_speech = len(__filename__) - int(str_split[1]) - 1
        if which_speech < 0 or which_speech >= len(__filename__):
            print("invalid continue command")
            return
        del __filename__[0:which_speech]
        del __begin__[0:which_speech]
        __wf__ = wave.open(__filename__[0], "r")

    # data.data = "interrupt:filename, start, end, amp, speed"
    elif str_split[0].startswith("interrupt:"):
        interrupt_split = str_split[0].split(":")
        filename = subprocess.Popen(
                        'echo $(rospack find aria_2ndparty)/sound/voice/',
                        shell=True,
                        stdout=subprocess.PIPE)
        filename = filename.communicate()[0]
        filename = re.split('\n', filename)
        filename = filename[0]
        __filename__.insert(0, "".join((filename,interrupt_split[1])))
        __wf__ = wave.open(__filename__[0], "r")
        __begin__.insert(0, float(str_split[1])*__wf__.getframerate())

    # data.data = "filename, start, end, amp, speed"
    else:
        filename = subprocess.Popen(
                        'echo $(rospack find aria_2ndparty)/sound/voice/',
                        shell=True,
                        stdout=subprocess.PIPE)
        filename = filename.communicate()[0]
        filename = re.split('\n', filename)
        filename = filename[0]
        __filename__[0] = "".join((filename,str_split[0]))
        __wf__ = wave.open(__filename__[0], "r")
        __begin__[0] = float(str_split[1])*__wf__.getframerate()        

    if str_split[2] == "-1":
        end = __wf__.getnframes()
    else:
        end = float(str_split[2])*__wf__.getframerate()

    global __amp__
    __amp__ = float(str_split[3])
    speed = float(str_split[4])

    global __p__
    global __stream__
    __p__ = pyaudio.PyAudio()
    __stream__ = __p__.open(format=__p__.get_format_from_width(__wf__.getsampwidth()),
                            channels=__wf__.getnchannels(),
                            rate=int(__wf__.getframerate()*speed),
                            output=True)
    global __play_range__
    global __chunk__
    global __count__
    __play__ = True
    __play_range__ = int((end - __begin__[0])/__chunk__)
    __count__ = 0
    play_data = __wf__.readframes(int(__begin__[0]))


if __name__ == '__main__':
    rospy.init_node('wav_handler', anonymous=True)
    rospy.Subscriber("py_voice", String, callback)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if __play__:
            play_data = __wf__.readframes(__chunk__)
            decoded_data = numpy.fromstring(play_data, numpy.int16)
            amplified_data = (decoded_data*__amp__).astype(numpy.int16)
            __stream__.write(amplified_data.tostring())
            __begin__[0] += __chunk__
            __count__ += 1
            if __count__ > __play_range__:
                __stream__.close()
                __p__.terminate()
                __play__ = False
                __count__ = 0
        else:
            r.sleep()

