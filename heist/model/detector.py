import random

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


    
def _label_to_color(label):
    random.seed(label)  # Use label as seed to generate a stable color
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return (r, g, b)

def get_yellow_regions(image_bgr,
                       lower_hsv=(20, 100, 100),
                       upper_hsv=(35, 255, 255),
                       min_area=500):
    """
    image_bgr: H×W×3 BGR numpy array
    Returns a list of (x, y, w, h) boxes for yellow blobs ≥ min_area.
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    # clean small specks
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # find contours
    cnts, _ = cv2.findContours(mask,
                               cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if w * h >= min_area:
            boxes.append((x, y, w, h))
    return boxes

def _box_iou(b1, b2):
    x1,y1,x2,y2 = b1
    x1b,y1b,x2b,y2b = b2
    xi1, yi1 = max(x1, x1b), max(y1, y1b)
    xi2, yi2 = min(x2, x2b), min(y2, y2b)
    inter_w = max(0, xi2 - xi1)
    inter_h = max(0, yi2 - yi1)
    inter = inter_w * inter_h
    area1 = (x2 - x1) * (y2 - y1)
    area2 = (x2b - x1b) * (y2b - y1b)
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0.0
    
        
def _simple_nms(preds, iou_thresh=0.5):
        """
        preds: list of ((x1,y1,x2,y2), label)
        returns: filtered list with high‐IoU duplicates removed
        """
        keep = []
        for box, label in preds:
            if not any(
                label == lbl and _box_iou(box, obox) > iou_thresh
                for obox, lbl in keep
            ):
                keep.append((box, label))
        return keep
class Detector:
    def __init__(self, yolo_dir="/root/yolo", from_tensor_rt=True, threshold=0.4):
        # local import
        from ultralytics import YOLO
        cls = YOLO
        
        self.threshold = threshold
        self.yolo_dir = yolo_dir
        if from_tensor_rt:
            self.model = cls(f"{self.yolo_dir}/yolo11n.engine", task="detect")
        else:
            self.model = cls(f"{self.yolo_dir}/yolo11n.pt", task="detect")
    
    def to(self, device):
        self.model.to(device)
        

    def predict(self, img, silent=True):
        """
        Note: img can be any of the following:
            Union[str, pathlib.Path, int, PIL.Image.Image, list, tuple, numpy.ndarray, torch.Tensor]

            Batch not supported.
            
        Runs detection on a single image and returns a list of
        ((xmin, ymin, xmax, ymax), class_label) for each detection
        above the given confidence threshold.
        """
        results = list(self.model(img, verbose=not silent))[0]
        boxes = results.boxes

        predictions = []
        # Iterate over the bounding boxes
        for xyxy, conf, cls_idx in zip(boxes.xyxy, boxes.conf, boxes.cls):
            if conf.item() >= self.threshold:
                # Convert bounding box tensor to Python floats
                x1, y1, x2, y2 = xyxy.tolist()
                # Map class index to class label using model/ results
                label = results.names[int(cls_idx.item())]
                predictions.append(((x1, y1, x2, y2), label))
        
        #convert original image to rgb
        original_image = results.orig_img
        cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB, original_image)

        return dict(predictions=predictions, original_image=original_image)
    
    def set_threshold(self, threshold):
        """
        Sets the confidence threshold for predictions.
        """
        self.threshold = threshold

    def draw_box(self, img, predictions, draw_all=False):
        """
        Draw bounding boxes on 'img'.

        :param img: The image to draw on (PIL.Image or NumPy array).
        :param predictions: A list of ((xmin, ymin, xmax, ymax), label).
        :param draw_all: If True, draw *all* bounding boxes.
                         If False, draw only the first one.
        :return: A PIL image with boxes and labels drawn.
        """
        if not predictions:
            return img  # No detections, return as is

        # Convert to PIL.Image if needed
        if not isinstance(img, Image.Image):
            img = Image.fromarray(img)

        draw = ImageDraw.Draw(img)

        min_dim = min(img.width, img.height)
        scale_factor = (
            min_dim / 600.0
        )

        line_width = max(
            1, int(4 * scale_factor)
        )
        font_size = max(10, int(20 * scale_factor))
        text_offset = font_size + 3

        try:
            font = ImageFont.truetype("arial.ttf", font_size)
        except IOError:
            font = ImageFont.load_default()

        print(f"Labels: {[x[-1] for x in predictions]}")

        if draw_all:
            for (x1, y1, x2, y2), label in predictions:
                color = _label_to_color(label)
                draw.rectangle([x1, y1, x2, y2], outline=color, width=line_width)
                draw.text((x1, y1 - text_offset), label, fill=color, font=font)
        else:
            (x1, y1, x2, y2), label = predictions[0]
            color = _label_to_color(label)
            draw.rectangle([x1, y1, x2, y2], outline=color, width=line_width)
            draw.text((x1, y1 - text_offset), label, fill=color, font=font)

        return img
    
    def id2name(self, i):
        """
        Converts a class index to a class name.
        """
        return self.model.names[i]
    
    @property
    def classes(self):
        return self.model.names
    

     
    
def demo():
    import os
    import numpy as np
    from PIL import Image

    model = Detector(
        yolo_dir="src/final_challenge2025/shrinkray_heist/model",
        from_tensor_rt=False
    )
    model.set_threshold(0.4)
    
    img_path = f"{os.path.dirname(__file__)}/../../media/minion.png" 



    img = Image.open(img_path)
    results = model.predict(img)
    
    predictions = results["predictions"]
    original_image = results["original_image"]

    out = model.draw_box(original_image, predictions, draw_all=True)
    
    save_path = f"{os.path.dirname(__file__)}/demo_output.png"
    out.save(save_path)
    print(f"Saved demo to {save_path}!")

if __name__ == '__main__':    
    demo()