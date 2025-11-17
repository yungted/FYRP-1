import cv2
import os
import time
from PIL import Image

# --- Moondream Setup ---
# NOTE: Replace with your actual API key
API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiIxZWI5Njk0MS02MDVlLTQ3ZTYtYjI5NS01NjdjMGRjYWZhYTkiLCJvcmdfaWQiOiJlZzJ2bXo4a1FWY0xlOTExQVl5bkM0NU44YkhEYTMwTyIsImlhdCI6MTc2MjkzMzIzOSwidmVyIjoxfQ.onaMxtqh-X14ggd8XUDfiALxzYuHuUyI3GMbWHBYCqw"

# Ensure you have the 'moondream' library installed: pip install moondream
try:
    import moondream as md
    # Connect to the cloud
    md_model = md.vl(endpoint="https://api.moondream.ai/v1", api_key=API_KEY)
    print("✅ Moondream model connected.")
    AI_ENABLED = True
except ImportError:
    print("⚠️ Moondream library not found. Run 'pip install moondream'. AI vision disabled.")
    AI_ENABLED = False
except Exception as e:
    # Use a generic exception handler for robustness
    print(f"⚠️ Failed to connect to Moondream API: {e}. AI vision disabled.")
    AI_ENABLED = False


# --- Camera Setup ---
camera_index = 1  
cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)

if not cap.isOpened():
    print(f"❌ Cannot open camera {camera_index}")
    exit()

# --- Main Loop Variables ---
ai_answer = "Press 'c' to capture a frame and run AI analysis."
temp_image_path = "temp_frame_capture.jpg"
capture_message = ""
MESSAGE_DURATION = 50 # Display message for about 50 frames

print("\n--- Starting Live Feed ---")
print("Press 'c' to capture image and analyze.")
print("Press 'q' to quit.")


while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to grab frame")
        break
        
    # Check for key presses
    key = cv2.waitKey(1) & 0xFF
    
    # --- Key Press Logic ---
    if key == ord('c'):
        if AI_ENABLED:
            print("📷 'c' pressed. Capturing and sending for analysis...")
            capture_message = "CAPTURING & ANALYZING..."
            MESSAGE_DURATION = 50 # Reset message duration

            # 1. Save the current frame
            cv2.imwrite(temp_image_path, frame)
            
            # 2. Load the image with PIL
            try:
                image = Image.open(temp_image_path)
            except Exception as e:
                print(f"Error loading image: {e}")
                ai_answer = f"Error loading image: {e}"
                continue

            # 3. Query the Moondream model
            try:
                # You can change the prompt here
                response = md_model.query(image, "Describe the main object or scene in this image in great detail.")
                new_answer = response.get("answer", "No answer returned.")
                ai_answer = f"AI: {new_answer}"
                print(f"✅ Analysis complete. Answer: {new_answer}")
                capture_message = "ANALYSIS COMPLETE!" 
            except Exception as e:
                ai_answer = f"API Error: {e}"
                print(f"❌ API Error: {e}")
                capture_message = "ANALYSIS FAILED!"
        else:
            capture_message = "AI DISABLED - Cannot analyze."
            MESSAGE_DURATION = 50
    
    # Press 'q' to quit
    if key == ord('q'):
        break
        
    # --- Display Frame, AI Text, and Status Message ---
    
    # 1. Add the AI's answer
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, ai_answer, (10, 30), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    
    # 2. Display the temporary capture/status message
    if MESSAGE_DURATION > 0:
        cv2.putText(frame, capture_message, (10, frame.shape[0] - 10), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        MESSAGE_DURATION -= 1 # Countdown the message display

    cv2.imshow("Live AI Camera Feed", frame)


# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
if os.path.exists(temp_image_path):
    os.remove(temp_image_path)
    print("🧹 Cleaned up temporary image file.")

print("Application closed.")
