#!/usr/bin/env python
import wave
import pyaudio
import re
import sys
import subprocess

if __name__ == '__main__':
    argv = sys.argv
    argc = len(argv)
    if argc != 2:
        quit()
    else:
        filename = subprocess.Popen(
            'echo $(rospack find aria_2ndparty)/sound/voice/',
            shell=True,
            stdout=subprocess.PIPE)
        filename = filename.communicate()[0]
        filename = re.split('\n', filename)
        filename = filename[0]
        wf = wave.open("".join((filename, argv[1])), "r")
        result = float(wf.getnframes()) / wf.getframerate()
        print result
