import cv2

camera_index = 1  # change to your desired camera index
cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)  # use DirectShow backend on Windows

if not cap.isOpened():
    print(f"⚠️ Cannot open camera {camera_index}")
    exit()

print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to grab frame")
        break

    cv2.imshow("Camera Feed", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
