import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask = cv2.erode(mask, kernel, iterations=1)

    blurred = cv2.GaussianBlur(mask, (5, 5), 0)
    edges = cv2.Canny(blurred, 100, 200)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    edge_contour_display = cv2.cvtColor(edges.copy(), cv2.COLOR_GRAY2BGR)
    cv2.drawContours(edge_contour_display, contours, -1, (0, 255, 0), 1)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:
            continue

        approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
        if len(approx) == 7:
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = float(area) / hull_area
            if solidity < 0.8:
                continue

            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            corners = approx
            farthest_pt = max(corners[:, 0, :], key=lambda point: np.linalg.norm(point - [cx, cy]))
            fx, fy = farthest_pt

            x, y, w, h = cv2.boundingRect(cnt)
            direction = ""
            if abs(fx - cx) > abs(fy - cy):
                direction = "RIGHT" if fx > cx else "LEFT"
            else:
                direction = "DOWN" if fy > cy else "UP"

            cv2.putText(frame, direction, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv2.imshow("Arrow Detection", frame)
    cv2.imshow("Edges + Contours", edge_contour_display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
