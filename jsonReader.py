#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

from subprocess import call
import os
import datetime
import shutil
from shutil import copytree, ignore_patterns
import json

# When using on the Pi, CHANGE Df to df. Case matters
command = "df -h > things.txt"

call([command], shell=True)

linesFromTxtFile = []

for line in open('things.txt'):
	linesFromTxtFile.append([line.rstrip('\n')])

portInformation = {}

for lists in linesFromTxtFile:
	listInfo = lists[0].split()
	if listInfo[0] not in portInformation:
		portInformation[listInfo[0]] = listInfo[-1]

# Change '/dev/disk2s1' to the port in the pi. For example my pi's port is '/dev/sda1'
portName = '/dev/sdb1'

files = os.listdir(portInformation[portName])

# Copies the file into the current directory
destination = os.getcwd()

# This is where the magic happens
try :
    for f in files:
        full_file_name = '{}/{}'.format(portInformation[portName], 'mar1.json')
        if (os.path.isfile(full_file_name)):
        	shutil.copy(full_file_name, destination)
except Exception as e:
    print(e)

# Read the json file
with open('mar1.json', encoding='utf-8') as data_file:
	data = json.loads(data_file.read())

print(data)
print('Size of arrays: {}'.format(data['size']))
print('X coords: {}'.format(data['x coords']))
print('Y coords: {}'.format(data['y coords']))