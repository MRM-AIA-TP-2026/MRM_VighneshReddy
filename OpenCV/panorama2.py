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

    cv2.imshow("Webcam", frame)
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
        captured_images[i] = cv2.resize(captured_images[i], (0, 0), fx=0.7, fy=0.7)

    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    result = captured_images[0]
    for i in range(1, 3):
        kp1, des1 = orb.detectAndCompute(result, None)
        kp2, des2 = orb.detectAndCompute(captured_images[i], None)
        
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)  
        
        h1, w1 = result.shape[:2]
        h2, w2 = captured_images[i].shape[:2]
        result = cv2.warpPerspective(result, M, (w1 + w2, h1))
        result[0:h2, 0:w2] = captured_images[i]
    
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        x, y, w, h = cv2.boundingRect(contours[0])
        result = result[y:y + h, x:x + w]

        target_width = int(h * 3)
        if w > target_width:
            crop_x = (w - target_width) // 2
            result = result[:, crop_x:crop_x + target_width]

    cv2.imshow('Final Panorama', result)
    cv2.imwrite("panorama.jpg", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Insufficient images captured.")
