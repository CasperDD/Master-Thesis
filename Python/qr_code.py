import cv2
from pyzbar import pyzbar


class QR:        

    def run_qr(self, img_src):
        # find and decode QR codes in the image
        barcodes = pyzbar.decode(img_src)

        detected_objects = []
        # loop over the detected barcodes
        for barcode in barcodes:
            # extract the bounding box location of the barcode
            (x, y, w, h) = barcode.rect

            # calculate the center x value of the bounding box
            center_x = x + w/2

            detected_objects.append((x, y, w, h))

        cv2.imshow('window', img_src)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            return None
        else:
            return detected_objects