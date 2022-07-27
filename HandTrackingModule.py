# -*- coding: utf-8 -*-
# modified from https://www.computervision.zone/courses/ai-virtual-mouse/
import cv2
import mediapipe as mp
import time
import math
 
 
class handDetector():
    def __init__(self, mode=False, maxHands=2, model_complexity=1, detectionCon=0.4, trackCon=0.4):
        self.mode = mode
        self.maxHands = maxHands
        self.model_complexity = model_complexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon
 
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.model_complexity,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
 
    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
 
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
 
        return img
 
    def findPosition(self, img, handNo=0, draw=True):
        xList = []
        yList = []
        bbox = []
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                xList.append(cx)
                yList.append(cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
 
            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax
 
            if draw:
                cv2.rectangle(img, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20),
                              (0, 255, 0), 2)
 
        return self.lmList, bbox
 
    def fingersUp(self):
        # 每次都会按照 拇指-食指-中指-无名指-小拇指的顺序检查是否伸出，对应返回列表的[0]-[4]
        fingers = []
        # Thumb
        if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 1][1]:  # 对拇指，比较p4和p2
            fingers.append(1)
        else:
            fingers.append(0)
 
        # Fingers
        for id in range(1, 5):
 
            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                # 对食指，比较P8和P6
                # 对中指，比较P12和P10
                # 对无名指，比较P16和P14
                # 对小拇指，比较P20和P18
                fingers.append(1)
            else:
                fingers.append(0)
 
        return fingers
 
    def findDistance(self, p1, p2, img, draw=True,r=15, t=3):
        x1, y1 = self.lmList[p1][1:]
        x2, y2 = self.lmList[p2][1:]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
 
        if draw:
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), t)
            cv2.circle(img, (x1, y1), r, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), r, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (cx, cy), r, (0, 0, 255), cv2.FILLED)
        length = math.hypot(x2 - x1, y2 - y1)
 
        return length, img, [x1, y1, x2, y2, cx, cy]
 
 
# def main():
#     pTime = 0
#     cap = cv2.VideoCapture(0)
#     detector = handDetector()
#     while cap.isOpened():
#         success, img = cap.read()
#         if not success:
#             print("No Frame!!")
#             continue
#         img.flags.writeable = False
#         img = detector.findHands(img)
#         lmList, bbox = detector.findPosition(img)
#         if len(lmList) != 0:
#             print(lmList[4])
 
#         cTime = time.time()
#         fps = 1 / (cTime - pTime)
#         pTime = cTime

#         img = cv2.flip(img, 1)
#         cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
#                     (255, 0, 255), 3)

#         cv2.imshow("Image", img)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     cap.release()
 
# if __name__ == "__main__":
#     main()
