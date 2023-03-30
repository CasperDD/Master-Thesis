#Code inspired by: https://opencv-tutorial.readthedocs.io/en/latest/yolo/yolo.html
import cv2
import numpy as np


class YOLO:
    def __init__(self):
        self.classes_names = open('coco.names').read().strip().split('\n')
        np.random.seed(42)
        self.colors_rnd = np.random.randint(0, 255, size=(len(self.classes_names), 3), dtype='uint8')

        self.net_yolo = cv2.dnn.readNetFromDarknet('yolov4-tiny.cfg', '/home/pi/yolov4-tiny.weights')
        self.net_yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net_yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.ln = self.net_yolo.getLayerNames()
        self.ln = [self.ln[i - 1] for i in self.net_yolo.getUnconnectedOutLayers()]

    def run_yolo(self, img_src):
        blob_img = cv2.dnn.blobFromImage(img_src, 1/255.0, (96, 96), swapRB=True, crop=False)
        r_blob = blob_img[0, 0, :, :]

        self.net_yolo.setInput(blob_img)
        outputs = self.net_yolo.forward(self.ln)

        boxes = []
        confidences = []
        classIDs = []
        h, w = img_src.shape[:2]

        for output in outputs:
            for detection in output:
                scores_yolo = detection[5:]
                classID = np.argmax(scores_yolo)
                confidence = scores_yolo[classID]
                if confidence > 0.4:
                    box_rect = detection[:4] * np.array([w, h, w, h])
                    (centerX, centerY, width, height) = box_rect.astype("int")
                    x_c = int(centerX) # - (width / 2)
                    y_c = int(centerY) #  - (height / 2)
                    box_rect = [x_c, y_c, int(width), int(height)]
                    boxes.append(box_rect)
                    confidences.append(float(confidence))
                    classIDs.append(classID)
                    # print("x_c: ", x_c)

        indices_yolo = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        detected_objects = []
        if len(indices_yolo) > 0:
            for i in indices_yolo.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                detected_objects.append((x, y, w, h))
                color = [int(c) for c in self.colors_rnd[classIDs[i]]]
                cv2.rectangle(img_src, (x, y), (x + w, y + h), color, 3)
                text = "{}: {:.4f}".format(self.classes_names[classIDs[i]], confidences[i])
                cv2.putText(img_src, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow('window', img_src)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            return None
        else:
            return detected_objects
