# -*- coding: utf-8 -*-
import numpy as np
import HandTrackingModule as htm
import time
import pyautogui
import cv2

##########################
# 显示框大小
wCam, hCam = 720, 405
# 有效框大小（frameRX-50 × hCam-50-frameRY）横坐标有考虑到镜像
frameRX = 200
frameRY = 200
pyautogui.FAILSAFE = False
pyautogui.PAUSE = 0.0  # 控制每条pyautogui指令执行的间隔
#########################
 
pTime = 0
plocX, plocY = 0, 0  # 鼠标之前的位置
clocX, clocY = 0, 0  # 鼠标现在的位置
smoothening = 10  # 移动量的衰减率，越大则鼠标移动地越慢
fps = 0.0

# 食指前后两帧的位置
prevIF_y = -1
currIF_y = -1
thresholdIF = 0.6  # 食指的速率阈值

# 中指前后两帧的位置
prevMF_y = -1
currMF_y = -1
thresholdMF = 0.6  # 中指的速率阈值

# 掌心前后两帧的位置
prevPalm_y = -1
currPalm_y = -1
thresholdPalm = 0.2

# 拖拽等待
drag_wait = 0
# 拖拽标记
drag_flag = 0

cap = cv2.VideoCapture(0)
cap.set(3, wCam)  # 设置帧宽度
cap.set(4, hCam)  # 设置帧高度
detector = htm.handDetector(maxHands=1)
wScr, hScr = pyautogui.size()

while cap.isOpened():
    success, img = cap.read()
    if not success:
        continue

    # 得到所有21个手部特征点
    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img) # lmList[finger_index][finger_index, x, y]
    # xmin,ymin,xmax,ymax
    cv2.rectangle(img, (50, frameRY), (frameRX, hCam - 100), (255, 0, 255), 2)

    if len(lmList) != 0:
        # 检查这一帧里哪几根手指伸出来了
        fingers = detector.fingersUp()

        # 设置滚轮模式
        # 同时伸出食指和中指为向上滚（这样可以防止收中指时误触右键），只伸出中指为向下滚
        if fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 0 and fingers[4] == 0:
            pyautogui.scroll(10)
            continue
        if fingers[1] == 0 and fingers[2] == 1 and fingers[3] == 0 and fingers[4] == 0:
            pyautogui.scroll(-10)
            continue

        # 设定鼠标移动模式：当只有食指伸出时，进入鼠标移动模式
        p0_x, p0_y = lmList[0][1:]  # 腕部坐标
        p9_x, p9_y = lmList[9][1:]  # 中指根部坐标
        p8_x, p8_y = lmList[8][1:]  # 食指指尖位置
        p12_x, p12_y = lmList[12][1:]  # 中指指尖位置
        md_x, md_y = (p0_x + p9_x) // 2, (p0_y + p9_y) // 2  # 求掌心坐标

        # 通过设置掌心移动速率阈值来区隔移动鼠标操作和点击操作
        currPalm_y = md_y
        currIF_y = p8_y
        currMF_y = p12_y
        if prevPalm_y >= 0 and fps > 0.0:
            if (currPalm_y - prevPalm_y) * fps / hCam < thresholdPalm:  # 此时认为手掌没有动，可以进行点击操作
                # 通过食指下按的速率来实现单击左键操作
                if prevIF_y >= 0:
                    if (currIF_y - prevIF_y) * fps / hCam > thresholdIF:
                        if fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:  # 防收起所有手指时误触
                            pyautogui.click()

                # 通过中指下按的速率来实现单击右键操作
                if prevMF_y >= 0:
                    if (currMF_y - prevMF_y) * fps / hCam > thresholdMF:
                        if fingers[1] == 1 and fingers[3] == 1 and fingers[4] == 1:  # 防收起所有手指时误触
                            pyautogui.click(button='RIGHT')
        prevPalm_y = currPalm_y
        prevIF_y = currIF_y
        prevMF_y = currMF_y

        # 移动模式
        if fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:
            if fingers[1] == 1:
                drag_wait = 0
                if drag_flag == 1:
                    pyautogui.mouseUp()  # 若之前有按下鼠标拖拽，则现在解除
                    drag_flag = 0
            # 将鼠标位置的屏幕坐标转换成有效框坐标
            md_x2 = np.interp(md_x, (50, frameRX), (0, wScr))
            md_y2 = np.interp(md_y, (frameRY, hCam - 100), (0, hScr))
            clocX = plocX + (md_x2 - plocX) / smoothening  # 右边那一项是移动量，即当前坐标减去之前坐标
            clocY = plocY + (md_y2 - plocY) / smoothening
            # 移动鼠标
            pyautogui.moveTo(wScr - clocX, clocY)  # 用duration参数由于使用了time.sleep()会导致fps下降，所以通过smoothing降低速度来平滑鼠标的移动
            cv2.circle(img, (md_x, md_y), 5, (255, 145, 100), cv2.FILLED)  # 在掌心位置画个点，表示掌心在指挥鼠标
            plocX, plocY = clocX, clocY  # 更新之前坐标为当前坐标

        # 实现拖拽
        if fingers[1] == 0 and fingers[2] == 1 and fingers[3] == 1 and fingers[4] == 1:
            if drag_flag == 0:
                drag_wait += 1
                if drag_wait > 10:
                    pyautogui.mouseDown()
                    drag_flag = 1
                    drag_wait = 0

    # fps
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    img = cv2.flip(img, 1)
    cv2.putText(img, str(int(fps)), (10, 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

    # 显示
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
