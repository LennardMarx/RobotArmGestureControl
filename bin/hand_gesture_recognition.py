#!/home/lennard/Projects/RobotArmGestureControl/venv2/bin/python
#!/usr/bin/env python3

# import necessary packages
import cv2
import numpy as np
import mediapipe as mp
# import tensorflow as tf
# import subprocess
#
# venv_path = '/home/lennard/Projects/RobotArmGestureControl/venv2/bin/activate'
#
# # Use the activate script directly
# activate_command = f'source {venv_path} && echo $VIRTUAL_ENV'
#
# # Execute the command
# result = subprocess.run(activate_command, shell=True,
#                         executable='/bin/bash', stdout=subprocess.PIPE, encoding='utf-8')
#
# python_bin = "/path/to/virtualenv/bin/python"
#
# # Path to the script that must run under the virtualenv
# script_file = "must/run/under/virtualenv/script.py"
#
# subprocess.Popen([python_bin, script_file])


# initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils


def fingerDistance(lm):
    try:
        return int(np.linalg.norm(np.array(lm[8])-np.array(lm[4])))
    except:
        return 0


cap = cv2.VideoCapture(0)


def run():
    # Initialize the webcam
    landmarks = []

    # Read each frame from the webcam
    _, frame = cap.read()
    x, y, c = frame.shape

    # Flip the frame vertically
    frame = cv2.flip(frame, 1)
    framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Get hand landmark prediction
    result = hands.process(framergb)
    # black = np.zeros_like(frame)

    # post process the result
    if result.multi_hand_landmarks:
        landmarks = []
        for handslms in result.multi_hand_landmarks:
            for lm in handslms.landmark:
                # print(id, lm)
                lmx = int(lm.x * x)
                lmy = int(lm.y * y)
                landmarks.append([lmx, lmy])
            # Drawing landmarks on frames
            # mpDraw.draw_landmarks(black, handslms, mpHands.HAND_CONNECTIONS)

    placeholder_list = [[0, 0] for _ in range(21)]
    if len(landmarks) == 21:
        return landmarks
    else:
        return placeholder_list


def cleanUp():
    # release the webcam and destroy all active windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    run()
