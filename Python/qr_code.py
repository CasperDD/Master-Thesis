# import cv2
# from pyzbar import pyzbar


# class QR:
#     def run_qr(self, img_src):
#         # find and decode QR codes in the image
#         barcodes = pyzbar.decode(img_src)

#         detected_objects = []
#         # loop over the detected barcodes
#         for barcode in barcodes:
#             # extract the bounding box location of the barcode
#             (x, y, w, h) = barcode.rect

#             # calculate the center x value of the bounding box
#             center_x = x + w/2

#             detected_objects.append((x, w, h))

#         cv2.imshow('window', img_src)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             cv2.destroyAllWindows()
#             return None
#         else:
#             return detected_objects


import threading
import cv2
from pyzbar import pyzbar


class QRCodeDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_ANY)
        self.frame = None
        self.qr_codes = set()
        self.frame_width = 160
        self.frame_height = 120

    def start(self):
        # Create a new thread to display the window
        threading.Thread(target=self._show_window, daemon=True).start()

        while True:
            # Read a frame from the camera
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Resize the frame to reduce processing time
            frame = cv2.resize(frame, (self.frame_width, self.frame_height))

            # Find QR codes in the frame
            qr_codes = set()
            barcodes = pyzbar.decode(frame)
            for barcode in barcodes:
                qr_codes.add(barcode.data.decode("utf-8"))

            # Update the current frame and QR codes
            self.frame = frame
            self.qr_codes = qr_codes

    def _show_window(self):
        cv2.namedWindow("QR Code Detector", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("QR Code Detector", self.frame_width, self.frame_height)

        while True:
            # If we have a frame, show it in the window
            if self.frame is not None:
                cv2.imshow("QR Code Detector", self.frame)

            # Wait for a key press and handle it
            key = cv2.waitKey(1)
            if key == ord("q"):
                break

        # Clean up
        cv2.destroyAllWindows()
        self.cap.release()

    def get_qr_codes(self):
        return self.qr_codes

qr_detector = QRCodeDetector()

qr_detector.start()

# Use the QR codes detected by the detector
qr_codes = qr_detector.get_qr_codes()
print(qr_codes)