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



import cv2
import threading
from pyzbar.pyzbar import decode
from camera import Camera

class QRCodeDetector:
    def __init__(self):
        self.frame = None
        self.qr_codes = []
        self.camera = Camera()

    def start(self):
        threading.Thread(target=self._run).start()

    def _run(self):
        # cv2.namedWindow("QR Code Detector", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("QR Code Detector", 160, 128)

        while True:
            if self.frame is not None:
                # Convert frame to grayscale and decode QR codes
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                self.qr_codes = decode(gray)

                # Display frame with detected QR codes
                for code in self.qr_codes:
                    x, y, w, h = code.rect
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(self.frame, code.data.decode("utf-8"), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                cv2.imshow("QR Code Detector", cv2.resize(self.frame, (160, 128)))
                self.frame = None

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()

    def detect_qr_codes(self, frame):
        self.frame = frame

    def get_qr_codes(self):
        return self.qr_codes