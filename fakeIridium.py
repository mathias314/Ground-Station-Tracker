import csv
import time
from datetime import datetime


def getTimeDiffs():
    flightData = open('flightData.csv', "r")

    csvReader = csv.reader(flightData)
    next(csvReader)

    firstLine = next(csvReader)
    # print(firstLine[1])
    oldTime = datetime.strptime(firstLine[1], "%Y-%m-%dT%H:%M:%SZ")
    # oldTime = 0
    print(oldTime)

    i = 0
    fullTimeDiff = []

    for line in csvReader:
        uglyDate = line[1]
        currTime = datetime.strptime(uglyDate, "%Y-%m-%dT%H:%M:%SZ")
        print((currTime - oldTime).total_seconds())
        fullTimeDiff.append( (currTime - oldTime).total_seconds() )

        oldTime = currTime
        i += 1

        print(i)

    flightData.close()

    return fullTimeDiff
