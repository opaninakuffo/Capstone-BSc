# Author: Opanin Akuffo
# Capstone Project
# Version 1.0
# 12/02/2021

import Leap
import sys
import time
import serial
import threading
from collections import deque

print("Initializing queue")
Lq = deque()
Rq = deque()
q = deque()
port1 = 'COM4'
port2 = 'COM5'
L_bluetooth = serial.Serial(port1, 9600)
R_bluetooth = serial.Serial(port2, 9600)
print('Right and Left arm bluetooth modules connected')
L_bluetooth.reset_input_buffer()
R_bluetooth.reset_input_buffer()

R_wristMin = 200
R_wristMax = 500
R_elbowMin = -100
R_elbowMax = 170
R_shoulderMin = 50
R_shoulderMax = 400

L_wristMin = 200
L_wristMax = 500
L_elbowMin = -100
L_elbowMax = 170
L_shoulderMin = -400
L_shoulderMax = -50

# Turn on dispatcher
dispatcher = 1


class Listener(Leap.Listener):

    def on_init(self, controller):
        print("Initializing")

    def on_connect(self, controller):
        print("Leap Motion connected")

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print("Leap Motion disconnected")

    def on_exit(self, controller):
        print("Exited")

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # Get hands
        for hand in frame.hands:
            if hand.confidence > 0.8 and abs(hand.palm_velocity.x) < 700 and abs(hand.palm_velocity.x) < 700 and abs(
                    hand.palm_velocity.x) < 700:
                if hand.is_right:
                    R_grip = hand.grab_strength

                    if hand.palm_position.y > R_wristMax:
                        R_wrist = R_wristMax
                    elif hand.palm_position.y < R_wristMin:
                        R_wrist = R_wristMin
                    else:
                        R_wrist = hand.palm_position.y

                    if hand.palm_position.z > R_elbowMax:
                        R_elbow = R_elbowMax
                    elif hand.palm_position.z < R_elbowMin:
                        R_elbow = R_elbowMin
                    else:
                        R_elbow = hand.palm_position.z

                    if hand.palm_position.x > R_shoulderMax:
                        R_shoulder = R_shoulderMax
                    elif hand.palm_position.x < R_shoulderMin:
                        R_shoulder = R_shoulderMin
                    else:
                        R_shoulder = hand.palm_position.x
                    # print("Hand: R, Grip: {}, Wrist: {}, Elbow: {}, Shoulder: {}".
                    # format(R_grip, R_wrist, R_elbow, R_shoulder))
                else:
                    L_grip = hand.grab_strength

                    if hand.palm_position.y > L_wristMax:
                        L_wrist = L_wristMax
                    elif hand.palm_position.y < L_wristMin:
                        L_wrist = L_wristMin
                    else:
                        L_wrist = hand.palm_position.y

                    if hand.palm_position.z > L_elbowMax:
                        L_elbow = L_elbowMax
                    elif hand.palm_position.z < L_elbowMin:
                        L_elbow = L_elbowMin
                    else:
                        L_elbow = hand.palm_position.z

                    if hand.palm_position.x > L_shoulderMax:
                        L_shoulder = L_shoulderMax
                    elif hand.palm_position.x < L_shoulderMin:
                        L_shoulder = L_shoulderMin
                    else:
                        L_shoulder = hand.palm_position.x
                    # print("Hand: L, Grip: {}, Wrist: {}, Elbow: {}, Shoulder: {}"
                    # .format(L_grip, L_wrist, L_elbow, L_shoulder))
        try:
            R_gripAngle = int(R_grip*135)
            R_wristAngle = int((R_wrist-R_wristMax)/(R_wristMin-R_wristMax)*180)
            R_elbowAngle = int((R_elbow-R_elbowMax)/(R_elbowMin-R_elbowMax)*180)
            R_shoulderAngle = int((R_shoulder-R_shoulderMax)/(R_shoulderMin-R_shoulderMax)*180)

            print("Hand: R, Grip: {}, Wrist: {}, Elbow: {}, Shoulder: {}".
                  format(R_gripAngle, R_wristAngle, R_elbowAngle, R_shoulderAngle))

            Rq.append(181)  # code for right gripper in MCU
            Rq.append(R_gripAngle)
            Rq.append(182)  # code for right wrist in MCU
            Rq.append(R_wristAngle)
            Rq.append(183)  # code for right elbow in MCU
            Rq.append(R_elbowAngle)
            Rq.append(184)  # code for right shoulder in MCU
            Rq.append(R_shoulderAngle)
        except UnboundLocalError:
            pass

        try:
            L_gripAngle = int(L_grip * 160)
            L_wristAngle = int((L_wrist - L_wristMax) / (L_wristMin - L_wristMax) * 180)
            L_elbowAngle = int((L_elbow - L_elbowMax) / (L_elbowMin - L_elbowMax) * 180)
            L_shoulderAngle = int((L_shoulder - L_shoulderMax) / (L_shoulderMin - L_shoulderMax) * 180)

            print("Hand: L, Grip: {}, Wrist: {}, Elbow: {}, Shoulder: {}".
                  format(L_gripAngle, L_wristAngle, L_elbowAngle, L_shoulderAngle))

            Lq.append(185)  # code for left gripper in MCU
            Lq.append(L_gripAngle)
            Lq.append(186)  # code for left wrist in MCU
            Lq.append(L_wristAngle)
            Lq.append(187)  # code for left elbow in MCU
            Lq.append(L_elbowAngle)
            Lq.append(188)  # code for left shoulder in MCU
            Lq.append(L_shoulderAngle)
        except UnboundLocalError:
            pass
        time.sleep(0.8)


def main():
    # Create a sample listener and controller
    listener = Listener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)
        # Disconnect bluetooth
        sys.exit(0)


def dispatch_right_arm_data():
    try:
        while True:
            if len(Rq):
                R_bluetooth.write(chr(Rq.popleft()).encode())
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        R_bluetooth.close()
        sys.exit(0)


def dispatch_left_arm_data():
    try:
        while True:
            if len(Lq):
                L_bluetooth.write(chr(Lq.popleft()).encode())
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        L_bluetooth.close()
        sys.exit(0)

# def dispatch_arm_data():
#     try:
#         while True:
#             if len(Lq):
#                 L_bluetooth.write(chr(Lq.popleft()).encode())
#                 time.sleep(0.08)
#                 L_bluetooth.write(chr(Lq.popleft()).encode())
#                 time.sleep(0.08)
#             if len(Rq):
#                 L_bluetooth.write(chr(Rq.popleft()).encode())
#                 time.sleep(0.08)
#                 L_bluetooth.write(chr(Rq.popleft()).encode())
#                 time.sleep(0.08)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         L_bluetooth.close()
#         sys.exit(0)


if __name__ == "__main__":
    threading.Thread(target=main).start()
    threading.Thread(target=dispatch_left_arm_data).start()
    threading.Thread(target=dispatch_right_arm_data).start()
