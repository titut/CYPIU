import cv2
import os

def fill_zero_num(num):
    num_zeros = 3 - len(str(num))
    return "0"*num_zeros + str(num)

# Open the default camera (index 0). 
# If you have multiple cameras, you might need to try different indices (1, 2, etc.).
cap = cv2.VideoCapture(4)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Loop to continuously read and display frames
while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # If the frame was not successfully read, break the loop
    if not ret:
        print("Error: Failed to grab frame.")
        break

    # Display the captured frame in a window named 'Camera Feed'
    cv2.imshow('Camera Feed', frame)

    # Wait for 1 millisecond and check for a key press.
    # If the 'q' key is pressed, exit the loop.
    key_pressed = cv2.waitKey(1)
    if key_pressed == ord('q'):
        break

    if key_pressed == ord('p'):
        all_entries = os.listdir("../img/raw")
        id = len(all_entries) + 1

        filename = f"../img/raw/image{fill_zero_num(id)}.jpg"

        # Save the frame as an image
        cv2.imwrite(filename, frame)
        print(f"Frame saved as {filename}")

# Release the camera and destroy all OpenCV windows
cap.release()
cv2.destroyAllWindows()