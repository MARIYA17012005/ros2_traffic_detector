import cv2
import numpy as np

def create_image(color, filename):
    img = np.zeros((300, 300, 3), dtype=np.uint8)
    if color == "red":
        cv2.circle(img, (150, 150), 50, (0, 0, 255), -1)
    elif color == "green":
        cv2.circle(img, (150, 150), 50, (0, 255, 0), -1)
    elif color == "empty":
        pass  # leave it black
    cv2.imwrite(filename, img)

create_image("red", "red-light.jpg")
create_image("green", "green-light.jpg")
create_image("empty", "empty-road.jpg")
print("Sample images generated.")
