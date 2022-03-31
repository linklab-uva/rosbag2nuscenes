import sys
import os
import numpy as np
import yaml
import csv

folder = "/home/john/Downloads/test.csv"
csvfile = open(folder)

reader = csv.reader(csvfile, delimiter=',', quotechar='"')
data = []
for row in reader:
    data.append(row)
data.pop(0)
print(data)

csvfile.close()
