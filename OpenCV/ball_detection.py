import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray_frame, (15, 15), 0)

    circles = cv2.HoughCircles(gray_blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=50, param2=30, minRadius=15, maxRadius=50)
    mask = np.zeros_like(gray_frame)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, radius = circle
            cv2.circle(mask, (x, y), radius, 255, -1)

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_hsv = cv2.inRange(frame_hsv, np.array([25, 100, 100]), np.array([35, 255, 255]))
    mask_combined = cv2.bitwise_and(mask, mask_hsv)

    mask_combined = cv2.medianBlur(mask_combined, 15)
    mask_combined = cv2.erode(mask_combined, None, iterations=2)
    mask_combined = cv2.dilate(mask_combined, None, iterations=2)

    circles = cv2.HoughCircles(mask_combined, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=50, param2=30, minRadius=15, maxRadius=50)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, radius = circle
            cv2.circle(frame, (x, y), radius, (0, 255, 0), 3)
            cv2.putText(frame, "Ball Detected", (x - 50, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask_combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
