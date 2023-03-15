# import cv2
# import argparse
# import numpy as np

# ap = argparse.ArgumentParser()
# ap.add_argument('-i', '--image', required=True,
#                 help = 'path to input image')
# ap.add_argument('-c', '--config', required=True,
#                 help = 'path to yolo config file')
# ap.add_argument('-w', '--weights', required=True,
#                 help = 'path to yolo pre-trained weights')
# ap.add_argument('-cl', '--classes', required=True,
#                 help = 'path to text file containing class names')
# args = ap.parse_args()


# def get_output_layers(net):
    
#     layer_names = net.getLayerNames()
#     try:
#         output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
#     except:
#         output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

#     return output_layers


# def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):

#     label = str(classes[class_id])

#     color = COLORS[class_id]

#     cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)

#     cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    
# image = cv2.imread(args.image)

# Width = image.shape[1]
# Height = image.shape[0]
# scale = 0.00392

# classes = None

# with open(args.classes, 'r') as f:
#     classes = [line.strip() for line in f.readlines()]

# COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

# net = cv2.dnn.readNet(args.weights, args.config)

# blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

# net.setInput(blob)

# outs = net.forward(get_output_layers(net))

# class_ids = []
# confidences = []
# boxes = []
# conf_threshold = 0.5
# nms_threshold = 0.4


# for out in outs:
#     for detection in out:
#         scores = detection[5:]
#         class_id = np.argmax(scores)
#         confidence = scores[class_id]
#         if confidence > 0.5:
#             center_x = int(detection[0] * Width)
#             center_y = int(detection[1] * Height)
#             w = int(detection[2] * Width)
#             h = int(detection[3] * Height)
#             x = center_x - w / 2
#             y = center_y - h / 2
#             class_ids.append(class_id)
#             confidences.append(float(confidence))
#             boxes.append([x, y, w, h])


# indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

# for i in indices:
#     try:
#         box = boxes[i]
#     except:
#         i = i[0]
#         box = boxes[i]
    
#     x = box[0]
#     y = box[1]
#     w = box[2]
#     h = box[3]
#     draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))

# cv2.imshow("object detection", image)
# cv2.waitKey()
    
# cv2.imwrite("object-detection.jpg", image)
# cv2.destroyAllWindows()





# import cv2
# import numpy as np

# img_src = cv2.imread('image.png')
# cv2.imshow('window',  img_src)
# cv2.waitKey(1)

# classes_names = open('coco.names').read().strip().split('\n')
# np.random.seed(42)
# colors_rnd = np.random.randint(0, 255, size=(len(classes_names), 3), dtype='uint8')

# net_yolo = cv2.dnn.readNetFromDarknet('yolov3.cfg', '/home/pi/yolov3.weights')
# net_yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
# net_yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# ln = net_yolo.getLayerNames()
# ln = [ln[i - 1] for i in net_yolo.getUnconnectedOutLayers()]

# blob_img = cv2.dnn.blobFromImage(img_src, 1/255.0, (416, 416), swapRB=True, crop=False)
# r_blob = blob_img[0, 0, :, :]

# cv2.imshow('blob', r_blob)
# text = f'Blob shape={blob_img.shape}'

# net_yolo.setInput(blob_img)
# outputs = net_yolo.forward(ln)

# boxes = []
# confidences = []
# classIDs = []
# h, w = img_src.shape[:2]

# for output in outputs:
#     for detection in output:
#         scores_yolo = detection[5:]
#         classID = np.argmax(scores_yolo)
#         confidence = scores_yolo[classID]
#         if confidence > 0.5:
#             box_rect = detection[:4] * np.array([w, h, w, h])
#             (centerX, centerY, width, height) = box_rect.astype("int")
#             x_c = int(centerX - (width / 2))
#             y_c = int(centerY - (height / 2))
#             box_rect = [x_c, y_c, int(width), int(height)]
#             boxes.append(box_rect)
#             confidences.append(float(confidence))
#             classIDs.append(classID)

# indices_yolo = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
# if len(indices_yolo) > 0:
#     for i in indices_yolo.flatten():
#         (x, y) = (boxes[i][0], boxes[i][1])
#         (w, h) = (boxes[i][2], boxes[i][3])
#         color = [int(c) for c in colors_rnd[classIDs[i]]]
#         cv2.rectangle(img_src, (x, y), (x + w, y + h), color, 3)
#         text = "{}: {:.4f}".format(classes_names[classIDs[i]], confidences[i])
#         cv2.putText(img_src, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

# cv2.imshow('window', img_src)
# cv2.waitKey(0)
# cv2.destroyAllWindows()





# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import cv2
# import numpy as np


# def imageFlip(image):
#     image = cv2.flip(image, 0)
#     image = cv2.flip(image, 1)
#     return image


# camera = PiCamera()
# rawCapture = PiRGBArray(camera, size=(640, 480))

# classes_names = open('coco.names').read().strip().split('\n')
# np.random.seed(42)
# colors_rnd = np.random.randint(0, 255, size=(len(classes_names), 3), dtype='uint8')

# net_yolo = cv2.dnn.readNetFromDarknet('yolov3.cfg', '/home/yolov3.weights')
# net_yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
# net_yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# ln = net_yolo.getLayerNames()
# ln = [ln[i - 1] for i in net_yolo.getUnconnectedOutLayers()]


# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#     # grab the raw NumPy array representing the image, then initialize the timestamp
#     # and occupied/unoccupied text
#     image = frame.array
#     # show the frame
#     image = imageFlip(image)
#     cv2.imshow("Frame", image)
#     key = cv2.waitKey(1) & 0xFF
#     # clear the stream in preparation for the next frame
#     rawCapture.truncate(0)

#     blob_img = cv2.dnn.blobFromImage(image, 1/255.0, (640, 480), swapRB=True, crop=False)
#     r_blob = blob_img[0, 0, :, :]

#     cv2.imshow('blob', r_blob)
#     text = f'Blob shape={blob_img.shape}'

#     net_yolo.setInput(blob_img)
#     outputs = net_yolo.forward(ln)

#     boxes = []
#     confidences = []
#     classIDs = []
#     h, w = image.shape[:2]

#     for output in outputs:
#         for detection in output:
#             scores_yolo = detection[5:]
#             classID = np.argmax(scores_yolo)
#             confidence = scores_yolo[classID]
#             if confidence > 0.5:
#                 box_rect = detection[:4] * np.array([w, h, w, h])
#                 (centerX, centerY, width, height) = box_rect.astype("int")
#                 x_c = int(centerX - (width / 2))
#                 y_c = int(centerY - (height / 2))
#                 box_rect = [x_c, y_c, int(width), int(height)]
#                 boxes.append(box_rect)
#                 confidences.append(float(confidence))
#                 classIDs.append(classID)

#     indices_yolo = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
#     if len(indices_yolo) > 0:
#         for i in indices_yolo.flatten():
#             (x, y) = (boxes[i][0], boxes[i][1])
#             (w, h) = (boxes[i][2], boxes[i][3])
#             color = [int(c) for c in colors_rnd[classIDs[i]]]
#             cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)
#             text = "{}: {:.4f}".format(classes_names[classIDs[i]], confidences[i])
#             cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

#     cv2.imshow('window', image)

#     # if the `q` key was pressed, break from the loop
#     if key == ord("q"):
#         break
    
#     cv2.waitKey(1)

# cv2.destroyAllWindows()





import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera


def imageFlip(image):
        image = cv2.flip(image, 0)
        image = cv2.flip(image, 1)
        return image


camera = PiCamera()
camera.resolution = (160, 120)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=camera.resolution)

classes_names = open('coco.names').read().strip().split('\n')
np.random.seed(42)
colors_rnd = np.random.randint(0, 255, size=(len(classes_names), 3), dtype='uint8')

net_yolo = cv2.dnn.readNetFromDarknet('yolov3.cfg', '/home/pi/yolov3.weights')
net_yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net_yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

ln = net_yolo.getLayerNames()
ln = [ln[i - 1] for i in net_yolo.getUnconnectedOutLayers()]

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img_src = frame.array

    img_src = imageFlip(img_src)
    
    blob_img = cv2.dnn.blobFromImage(img_src, 1/255.0, (128, 128), swapRB=True, crop=False)
    r_blob = blob_img[0, 0, :, :]
    
    net_yolo.setInput(blob_img)
    outputs = net_yolo.forward(ln)

    boxes = []
    confidences = []
    classIDs = []
    h, w = img_src.shape[:2]

    for output in outputs:
        for detection in output:
            scores_yolo = detection[5:]
            classID = np.argmax(scores_yolo)
            confidence = scores_yolo[classID]
            if confidence > 0.5:
                box_rect = detection[:4] * np.array([w, h, w, h])
                (centerX, centerY, width, height) = box_rect.astype("int")
                x_c = int(centerX - (width / 2))
                y_c = int(centerY - (height / 2))
                box_rect = [x_c, y_c, int(width), int(height)]
                boxes.append(box_rect)
                confidences.append(float(confidence))
                classIDs.append(classID)
                print("x_c: ", x_c)

    indices_yolo = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    if len(indices_yolo) > 0:
        for i in indices_yolo.flatten():
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            color = [int(c) for c in colors_rnd[classIDs[i]]]
            cv2.rectangle(img_src, (x, y), (x + w, y + h), color, 3)
            text = "{}: {:.4f}".format(classes_names[classIDs[i]], confidences[i])
            cv2.putText(img_src, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow('window', img_src)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rawCapture.truncate(0)

cv2.destroyAllWindows()