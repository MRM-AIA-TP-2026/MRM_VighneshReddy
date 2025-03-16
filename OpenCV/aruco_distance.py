import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)

aruco_dictionary_4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
aruco_dictionary_6x6 = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector_4x4 = aruco.ArucoDetector(aruco_dictionary_4x4, parameters)
detector_6x6 = aruco.ArucoDetector(aruco_dictionary_6x6, parameters)

camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners_4x4, ids_4x4, _ = detector_4x4.detectMarkers(gray)
    corners_6x6, ids_6x6, _ = detector_6x6.detectMarkers(gray)

    if ids_4x4 is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners_4x4, 0.1, camera_matrix, dist_coeffs)
        for i in range(len(rvecs)):
            depth = np.linalg.norm(tvecs[i][0])
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.3)
            center = np.mean(corners_4x4[i][0], axis=0).astype(int)
            #cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType)-->general syntax
            cv2.putText(frame, f"{depth:.2f} m", (center[0], center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"rvecs: {rvecs}")
            print(f"tvecs: {tvecs}")
        aruco.drawDetectedMarkers(frame, corners_4x4, ids_4x4)

    if ids_6x6 is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners_6x6, 0.1, camera_matrix, dist_coeffs)
        for i in range(len(rvecs)):
            depth = np.linalg.norm(tvecs[i][0])
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.3)
            center = np.mean(corners_6x6[i][0], axis=0).astype(int)
            cv2.putText(frame, f"{depth:.2f} m", (center[0], center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"rvecs: {rvecs}")
            print(f"tvecs: {tvecs}")
        aruco.drawDetectedMarkers(frame, corners_6x6, ids_6x6)

    cv2.imshow("Aruco Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
