import json

from functools import reduce

import argparse

class UnsafeRectangle:
    def __init__(self, top_y, right_x, left_x, bottom_y):
        self.top_y = top_y
        self.right_x = right_x
        self.left_x = left_x
        self.bottom_y = bottom_y

    def contains(self, x, y):
        return self.left_x <= x <= self.right_x and y >= self.bottom_y and y <= self.top_y


class SafetyTest:
    def __init__(self, map_filename):
        self.map_filename = map_filename
        self.unsafe_rectangles = self.loadUnsafeRectangles()

    def loadUnsafeRectangles(self):
        unsafe_rectangles = []
        with open(self.map_filename) as map_file:
            data = json.load(map_file)
            unsafe_rects = data["unsafe-rects"]
            for unsafe_rect in unsafe_rects:
                top_y = unsafe_rect["top_y"]
                right_x = unsafe_rect["right_x"]
                left_x = unsafe_rect["left_x"]
                bottom_y = unsafe_rect["bottom_y"]
                rect = UnsafeRectangle(top_y, right_x, left_x, bottom_y)
                unsafe_rectangles.append(rect)
        return unsafe_rectangles

    def isRobotSafe(self, bot_x, bot_y):
        for unsafe_rect in self.unsafe_rectangles:
            if unsafe_rect.contains(bot_x, bot_y):
                return False
        return True


def getIsSafeRecords(map_filename, time_pose_records):
    isSafeRecords = []
    safetyTest = SafetyTest(map_filename)
    for time, bot_x, bot_y in time_pose_records:
        isSafe = safetyTest.isRobotSafe(bot_x, bot_y)
        isSafeRec = (time, isSafe)
        isSafeRecords.append(isSafeRec)
    return isSafeRecords


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("map_filename")
#     args = parser.parse_args()
#     map_filename = args.map_filename
#     # TODO: This array is obtained from simulation run
#     time_pose_records = []
#     isSafeRecords = getIsSafeRecords(map_filename, time_pose_records)
#     isRunSafe = reduce((lambda rec_1, rec_2: rec_1[1] and rec_2[1]), isSafeRecords)
#     # TODO: Write isSafeRecords (for measurement_verbose table) and isRunsafe (measurement table) to database
