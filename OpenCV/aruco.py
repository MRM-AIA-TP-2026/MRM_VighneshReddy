import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)

aruco_dictionary_4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
aruco_dictionary_6x6 = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector_4x4 = aruco.ArucoDetector(aruco_dictionary_4x4, parameters)
detector_6x6 = aruco.ArucoDetector(aruco_dictionary_6x6, parameters)

camera_matrix = np.array([
    [5.526065834627128197e+02, 0.000000000000000000e+00, 3.142301240717363839e+02],
    [0.000000000000000000e+00, 5.526968420765390420e+02, 2.460328335477867654e+02],
    [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
], dtype=np.float32)

dist_coeffs = np.array([3.762641930176650190e-02, -9.302776835566305136e-02, 
                        -1.801525096588737444e-03, -1.170517979012785379e-04, 
                        -1.003357285412295186e+00], dtype=np.float32)

marker_size = 0.05

def euler(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners_4x4, ids_4x4, _ = detector_4x4.detectMarkers(gray)
    corners_6x6, ids_6x6, _ = detector_6x6.detectMarkers(gray)

    if ids_4x4 is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners_4x4, marker_size, camera_matrix, dist_coeffs)
        for i in range(len(rvecs)):
            depth = np.linalg.norm(tvecs[i][0])
            roll, pitch, yaw = euler(rvecs[i])
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.3)
            #cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType)-->general syntax
            corner = corners_4x4[i][0][0]  
            cv2.putText(frame, f"ID: {ids_4x4[i][0]}", (int(corner[0]), int(corner[1] - 10)), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            cv2.putText(frame, f"Depth: {depth:.2f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Roll: {roll:.2f}°", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Pitch: {pitch:.2f}°", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Yaw: {yaw:.2f}°", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        

    if ids_6x6 is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners_6x6, marker_size, camera_matrix, dist_coeffs)
        for i in range(len(rvecs)):
            depth = np.linalg.norm(tvecs[i][0])
            roll, pitch, yaw = euler(rvecs[i])
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.3)
            #cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType)-->general syntax
            corner = corners_6x6[i][0][0]  
            cv2.putText(frame, f"ID: {ids_6x6[i][0]}", (int(corner[0]), int(corner[1] - 10)), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Depth: {depth:.2f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Roll: {roll:.2f}°", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Pitch: {pitch:.2f}°", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"Yaw: {yaw:.2f}°", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Aruco Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
