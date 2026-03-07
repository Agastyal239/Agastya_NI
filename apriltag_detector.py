import cv2
import sys

def detect_apriltags():
    # Try all AprilTag dictionary variants
    april_dicts = [
        ("APRILTAG_36h11", cv2.aruco.DICT_APRILTAG_36h11),
        ("APRILTAG_36h10", cv2.aruco.DICT_APRILTAG_36h10),
        ("APRILTAG_25h9",  cv2.aruco.DICT_APRILTAG_25h9),
        ("APRILTAG_16h5",  cv2.aruco.DICT_APRILTAG_16h5),
    ]

    detectors = []
    for name, dict_id in april_dicts:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()

        # Improve detection sensitivity
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.02   # detect smaller tags
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.05
        params.minCornerDistanceRate = 0.01
        params.minDistanceToBorder = 1
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.01
        params.errorCorrectionRate = 0.6       # more lenient error correction

        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        detectors.append((name, detector))

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)

    # Set higher resolution for better detection
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("AprilTag Detector running... Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Enhance contrast using CLAHE
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        any_detected = False

        for family_name, detector in detectors:
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                any_detected = True
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, tag_id in enumerate(ids.flatten()):
                    corner = corners[i][0]
                    cx = int(corner[:, 0].mean())
                    cy = int(corner[:, 1].mean())

                    label = f"ID:{tag_id} ({family_name})"
                    # Black outline for readability
                    cv2.putText(frame, label, (cx - 40, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
                    cv2.putText(frame, label, (cx - 40, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

                    print(f"Detected AprilTag | Family: {family_name} | ID: {tag_id}")

                    if tag_id == 0:
                                
                        print("Detected room 101")


        if not any_detected:
            cv2.putText(frame, "No AprilTags detected", (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("AprilTag Detector", frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quitting...")
            break

    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    detect_apriltags()