#!/bin/bash
header='#!/usr/bin/env python\nimport aria\nfrom std_msgs.msg import String\n'
packages=''

for entry in ../motions/*.py
do
    tmp="${entry##*/}"
    packages="${packages}from ${tmp:0:${#tmp}-3} import *\n"
done

def='\naria.init_publisher()\n'
def="${def}motions = {"

for entry in ../motions/*.py
do
    tmp="${entry##*/}"
    def=$"${def}'${tmp:0:${#tmp}-3}': ${tmp:0:${#tmp}-3},"
done

def="${def:0:${#def}-1}"
def="${def}}\n"

body="$header$packages$def"
body="${body}\ndef callback(msg):\n\tmotions[msg.data]()\n\n"

main="if __name__ == '__main__':"
main="${main}\n\trospy.init_node('motion_runner',anonymous=True)\n\trospy.Subscriber('/aria/commandline',String,callback)\n\trospy.spin()"

body="$body$main"

echo -e $body > gen_test.py