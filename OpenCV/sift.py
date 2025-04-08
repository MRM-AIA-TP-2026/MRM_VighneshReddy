import cv2
import numpy as np

cap = cv2.VideoCapture(0)

captured_images = []
print("Press 'c' to capture an image.")

while len(captured_images) < 5:
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

if len(captured_images) == 5:
    for i in range(len(captured_images)):
        captured_images[i] = cv2.resize(captured_images[i], (0, 0), fx=0.5, fy=0.5)

    sift = cv2.SIFT_create()
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    result = captured_images[0]
    for i in range(1, 5):
        kp1, des1 = sift.detectAndCompute(result, None)
        kp2, des2 = sift.detectAndCompute(captured_images[i], None)

        if des1 is None or des2 is None or len(des1) < 4 or len(des2) < 4:
            print("Feature detection failed on one of the images.")
            continue

        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        if len(matches) < 4:
            print("Not enough matches found for homography.")
            continue

        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if M is None:
            print("Homography calculation failed.")
            continue

        h1, w1 = result.shape[:2]
        h2, w2 = captured_images[i].shape[:2]

        # Expand the canvas to avoid black borders
        expanded_width = w1 + w2
        expanded_height = max(h1, h2)

        result_warped = cv2.warpPerspective(result, M, (expanded_width, expanded_height))

        # Create a mask for the new image
        mask = np.zeros_like(result_warped, dtype=np.uint8)
        mask[0:h2, 0:w2] = 255  # Fill the mask with white for the new image area

        # Blending by combining images
        result_warped[0:h2, 0:w2] = captured_images[i]

        # Use the final result to combine images without black borders
        result = cv2.addWeighted(result_warped, 0.5, result, 0.5, 0)

    # Crop final output to remove any black borders
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        x, y, w, h = cv2.boundingRect(contours[0])
        result = result[y:y + h, x:x + w]

    cv2.imshow('Final Panorama', result)
    cv2.imwrite("output.jpg", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Insufficient images captured.")
