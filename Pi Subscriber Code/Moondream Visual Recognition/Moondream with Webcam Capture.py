#Moondream with Webcam Capture
import cv2
from PIL import Image
import io
import moondream as md

# Replace with your Moondream Cloud API key
API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiIxZWI5Njk0MS02MDVlLTQ3ZTYtYjI5NS01NjdjMGRjYWZhYTkiLCJvcmdfaWQiOiJlZzJ2bXo4a1FWY0xlOTExQVl5bkM0NU44YkhEYTMwTyIsImlhdCI6MTc2MjkzMzIzOSwidmVyIjoxfQ.onaMxtqh-X14ggd8XUDfiALxzYuHuUyI3GMbWHBYCqw"

# Connect to Moondream Cloud
model = md.vl(endpoint="https://api.moondream.ai/v1", api_key=API_KEY)

# Initialize webcam (0 = default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam.")

print("Press 'c' to capture an image, or 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Show the live camera feed
    cv2.imshow("Webcam", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("c"):
        # Capture triggered
        # Convert OpenCV BGR frame to RGB PIL Image
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(img_rgb)

        # Send to Moondream Cloud API
        response = model.query(image, "What do you see?")
        answer = response.get("answer", "No answer returned")
        print("Moondream says:", answer)

    elif key == ord("q"):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
