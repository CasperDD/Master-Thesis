import cv2
import numpy as np

# Load the model and the pre-trained weights
model = cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt.txt', '/home/pi/MobileNetSSD_deploy.caffemodel')

# Load the input image
image = cv2.imread('input_image.jpg')

# Preprocess the input image
blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

# Set the input for the model
model.setInput(blob)

# Run the model inference
detections = model.forward()


class_names = [['background',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor'],
          ['background', 'face']]

# Loop over the detections and draw boxes around the objects
for i in range(detections.shape[2]):
    confidence = detections[0, 0, i, 2]
    if confidence > 0.5:
        class_id = int(detections[0, 0, i, 1])
        class_name = class_names[class_id]
        box = detections[0, 0, i, 3:7] * np.array([image.shape[1], image.shape[0], image.shape[1], image.shape[0]])
        x1, y1, x2, y2 = box.astype('int')
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
        cv2.putText(image, class_name, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), thickness=2)

# Display the output image
cv2.imshow('MobileNet-SSD', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
