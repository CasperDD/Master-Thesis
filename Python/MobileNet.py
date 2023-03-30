import cv2
import numpy as np

class MobileNetSSD:
    def __init__(self):
        # Load MobileNet-SSD model
        self.net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt.txt", "/home/pi/MobileNetSSD_deploy.caffemodel")

        # Load class names
        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

    def detect(self, image):
        # Resize image to (300, 300)
        image = cv2.resize(image, (160, 120))

        # Preprocess image
        blob = cv2.dnn.blobFromImage(image, 0.007843, (160, 120), 127.5)

        # Set input
        self.net.setInput(blob)

        # Forward pass
        detections = self.net.forward()

        # Initialize empty list of detections
        results = []

        # Loop over detections
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            if confidence > 0.5:
                class_id = int(detections[0, 0, i, 1])
                class_name = self.classes[class_id]
                box = detections[0, 0, i, 3:7] * np.array([image.shape[1], image.shape[0], image.shape[1], image.shape[0]])
                (x, y, w, h) = box.astype("int")
                cv2.rectangle(image, (x, y), (w, h), (0, 255, 0), thickness=2)
                cv2.putText(image, class_name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), thickness=2)

                # Add detection to results list
                results.append((x + w/2, y + h/2))

    
        cv2.imshow('window', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            return None
        else:
            return results
