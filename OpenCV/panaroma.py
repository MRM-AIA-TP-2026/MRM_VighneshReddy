import cv2
import numpy as np

cap = cv2.VideoCapture(0)

captured_images = []
print("Press 'c' to capture an image.")

while len(captured_images) < 3:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame.")
        break

    cv2.imshow("Webcam: ", frame)

    key = cv2.waitKey(1)
    if key == ord('c'):
        print(f"Captured image {len(captured_images) + 1}")
        captured_images.append(frame)

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(captured_images) == 3:
    for i in range(len(captured_images)):
        captured_images[i] = cv2.resize(captured_images[i], (0, 0), fx=0.5, fy=0.5)

    cv2.imshow('Image 1', captured_images[0])
    cv2.imshow('Image 2', captured_images[1])
    cv2.imshow('Image 3', captured_images[2])

    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    kp1, des1 = orb.detectAndCompute(captured_images[0], None)
    kp2, des2 = orb.detectAndCompute(captured_images[1], None)
    matches1 = bf.match(des1, des2)
    matches1 = sorted(matches1, key=lambda x: x.distance)
    
    # #(N, 1, 2) N = number of points, 1 is single column dimension(for opencv) and 2 (each point has 2 values)
    
    src_pts1 = np.float32([kp1[m.queryIdx].pt for m in matches1]).reshape(-1, 1, 2)
    dst_pts1 = np.float32([kp2[m.trainIdx].pt for m in matches1]).reshape(-1, 1, 2)

    M1, mask1 = cv2.findHomography(src_pts1, dst_pts1, cv2.RANSAC, 3.0)
    h1, w1 = captured_images[0].shape[:2]
    h2, w2 = captured_images[1].shape[:2]
    result1 = cv2.warpPerspective(captured_images[0], M1, (w1 + w2, h1))
    result1[0:h2, 0:w2] = captured_images[1]

    kp3, des3 = orb.detectAndCompute(result1, None)
    kp4, des4 = orb.detectAndCompute(captured_images[2], None)
    matches2 = bf.match(des3, des4)
    matches2 = sorted(matches2, key=lambda x: x.distance)

    src_pts2 = np.float32([kp3[m.queryIdx].pt for m in matches2]).reshape(-1, 1, 2)
    dst_pts2 = np.float32([kp4[m.trainIdx].pt for m in matches2]).reshape(-1, 1, 2)

    M2, mask2 = cv2.findHomography(src_pts2, dst_pts2, cv2.RANSAC, 5.0)
    h3, w3 = captured_images[2].shape[:2]
    h4, w4 = result1.shape[:2]
    result2 = cv2.warpPerspective(result1, M2, (w4 + w3, h4))
    result2[0:h3, 0:w3] = captured_images[2]

    gray = cv2.cvtColor(result2, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        x, y, w, h = cv2.boundingRect(contours[0])
        result2 = result2[y:y + h, x:x + w]

    cv2.imshow('Final Panorama', result2)
    cv2.imwrite("output.jpg", result2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Insufficient images captured.")