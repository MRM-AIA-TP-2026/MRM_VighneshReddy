import cv2
import numpy as np

cap = cv2.VideoCapture(0)

frame_skip = 2
frame_count = 0

lower_gr = np.array([20, 60, 80])
upper_gr = np.array([40, 255, 255])

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    if frame_count % frame_skip != 0:
        continue

    resized_frame = cv2.resize(frame, (640, 480))

    blur = cv2.GaussianBlur(resized_frame, (15, 15), 0)
    frame_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask_hsv = cv2.inRange(frame_hsv, lower_gr, upper_gr)

    mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_OPEN, kernel, iterations=3)
    mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.GaussianBlur(mask_hsv, (15, 15), 0)

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.10, minDist=300,
                               param1=100, param2=35, minRadius=10, maxRadius=400)
    valid_detected = False

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            area = np.pi * radius**2
            if 500 < area < 50000:
                valid_detected = True
                cv2.circle(resized_frame, center, 1, (255, 0, 0), 3)
                cv2.circle(resized_frame, center, radius, (0, 255, 255), 3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if 0.8 < circularity < 1.2 and 500 < area < 50000:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                valid_detected = True
                cv2.circle(resized_frame, center, 1, (255, 0, 0), 3)
                cv2.circle(resized_frame, center, radius, (0, 255, 255), 3)

    if valid_detected:
        cv2.putText(resized_frame, "Ball Detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    cv2.imshow('frame', resized_frame)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()