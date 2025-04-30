import cv2
import numpy as np

class RaceCV():
    def __init__(self):
        self.out = None
        self.prev_lx = []
        self.prev_rx = []
        self.prev_left_fit = []
        self.prev_right_fit = []

    def show_video(self, img):
        """
        Show incoming images.
        """
        cv2.imshow("image", img)

        ### Continuous streaming ###
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    def record_video(self, img):
        """
        Record incoming images.
        """
        if img is None:
            if self.out is not None:
                self.out.release()
            return

        height, width, _ = img.shape
        fps = 30
        frame_size = (width, height)

        if self.out is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter('src/final/vids/run1.mp4', fourcc, fps, frame_size)

        self.out.write(img)

    def lane_following(self, frame):
        """
        Returns center offset given image of lane.
        """
        frame_height, frame_width, _ = frame.shape # (360, 640, 3)
        # frame = cv2.resize(frame, (640, 480)) # Resizing

        ### Area of focus ###
        tl = (frame_width // 2 - 170 + 30, 170)
        tr = (frame_width // 2 + 170, 170)
        bl = (30, 215)
        br = (640, 215)

        # Draw area of focus
        # cv2.circle(frame, tl, 5, (0, 0, 255), -1)
        # cv2.circle(frame, tr, 5, (0, 0, 255), -1)
        # cv2.circle(frame, bl, 5, (0, 0, 255), -1)
        # cv2.circle(frame, br, 5, (0, 0, 255), -1)

        pts1 = np.float32([tl, tr, bl, br])
        pts2 = np.float32([
            (0, 0),       # top-left
            (640, 0),     # top-right
            (0, 360),     # bottom-left
            (640, 360)    # bottom-right
        ])

        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        transformed_frame = cv2.warpPerspective(frame, matrix, (640, 360))

        ### Lane Detection ###
        hsv = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)

        lh = 0
        ls = 0
        lv = 200
        uh = 255
        us = 50
        uv = 255

        lower = np.array([lh, ls, lv])
        upper = np.array([uh, us, uv])
        mask = cv2.inRange(hsv, lower, upper)

        ### Mask processing ###
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(mask)

        for contour in contours:
            area = cv2.contourArea(contour)
            x_rect, y_rect, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h if h > 0 else 0

            if area < 300: # Filter by size
                continue

            cv2.drawContours(filtered_mask, [contour], -1, 255, -1)

        mask = filtered_mask

        horizontal_kernel = np.ones((1, 8), np.uint8)
        vertical_kernel = np.ones((5, 1), np.uint8)

        mask = cv2.dilate(mask, vertical_kernel, iterations=2)
        mask = cv2.erode(mask, horizontal_kernel, iterations=1)

        ### Histogram ###
        hist_offset = 110
        histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        left_base = np.argmax(histogram[:midpoint - hist_offset])
        right_base = np.argmax(histogram[midpoint + hist_offset:]) + midpoint + hist_offset

        ### Sliding window ###
        win_width = 50 # Half of width
        win_height = 40
        y = 360
        starting_y = y
        lane_width_meters = 0.4
        lx = []
        rx = []

        window_mask = mask.copy()
        while y > 0:
            # Left
            img = mask[max(0, y - win_height):y, max(0, left_base - win_width):left_base + win_width]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    lx.append(left_base - win_width + cx)
                    left_base = left_base - win_width + cx

            # Right
            img = mask[max(0, y - win_height):y, max(0, right_base - win_width):right_base + win_width]
            contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    rx.append(right_base - win_width + cx)
                    right_base = right_base - win_width + cx

            cv2.rectangle(window_mask, (left_base - win_width, y), (left_base + win_width, y - win_height), (255, 255, 255), 2)
            cv2.rectangle(window_mask, (right_base - win_width, y), (right_base + win_width, y - win_height), (255, 255, 255), 2)

            y -= win_height

        ### Lane coordinates ###
        if len(lx) == 0:
            lx = self.prev_lx
        else:
            self.prev_lx = lx
        if len(rx) == 0:
            rx = self.prev_rx
        else:
            self.prev_rx = rx
        min_length = min(len(lx), len(rx))

        if min_length == 0:
            print("No lanes detected")
            return None, None

        # Fit second order polynomial
        left_points = [(lx[i], y + i * win_height) for i in range(min_length)]
        right_points = [(rx[i], y + i * win_height) for i in range(min_length)]
        try:
            left_fit = np.polyfit([p[1] for p in left_points], [p[0] for p in left_points], 2)
            self.prev_left_fit = left_fit
        except (np.linalg.LinAlgError, ValueError):
            left_fit = self.prev_left_fit
        try:
            right_fit = np.polyfit([p[1] for p in right_points], [p[0] for p in right_points], 2)
            self.prev_right_fit = right_fit
        except (np.linalg.LinAlgError, ValueError):
            right_fit = self.prev_right_fit

        # Curvature
        y_eval = frame_height
        epsilon = 1e-6  # Small number to prevent divide-by-zero
        left_A = left_fit[0] if np.abs(left_fit[0]) > epsilon else epsilon
        right_A = right_fit[0] if np.abs(right_fit[0]) > epsilon else epsilon

        left_curvature = ((1 + (2 * left_fit[0] * y_eval + left_fit[1])**2)**1.5) / np.abs(2 * left_A)
        right_curvature = ((1 + (2 * right_fit[0] * y_eval + right_fit[1])**2)**1.5) / np.abs(2 * right_A)
        curvature = left_curvature + right_curvature / 2

        # Offset
        car_offset = 30 # Car offset
        lane_center = (left_base + right_base) / 2
        car_position = frame_width // 2 + car_offset
        lane_offset = (car_position - lane_center) * lane_width_meters / frame_width

        # Steering angle
        steering_angle = np.arctan(lane_offset / curvature)

        # Steering line
        line_length = 100
        end_x = int(frame_width // 2 + line_length * np.sin(steering_angle))
        end_y = int(frame_height - line_length * np.cos(steering_angle))

        ### Overlay ###
        top_left = (lx[0], starting_y)
        top_right = (rx[0], starting_y)
        bottom_left = (lx[min_length - 1], 0)
        bottom_right = (rx[min_length - 1], 0)

        quad_points = np.array([[top_left, top_right, bottom_right, bottom_left]], dtype=np.int32)
        quad_points = quad_points.reshape((-1, 1, 2))
        overlay = transformed_frame.copy()
        cv2.fillPoly(overlay, [quad_points], (0, 255, 0))
        alpha = 0.5
        cv2.addWeighted(overlay, alpha, transformed_frame, 1 - alpha, 0, transformed_frame)
        inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
        overlay_original = cv2.warpPerspective(transformed_frame, inv_matrix, (640, 360))
        result = cv2.addWeighted(frame, 1, overlay_original, 0.5, 0)

        ### Steering Messages ###
        cv2.line(result, (frame_width //2, frame_height), (end_x, end_y), (255, 0, 0), 2)
        cv2.putText(result, f'Curvature: {curvature:.2f} m', (30, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, f'Offset: {lane_offset:.2f} m', (30, 70), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, f'Angle: {steering_angle:.2f} m', (30, 110), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

        ### Visualization ###
        # cv2.imshow("Original", frame)
        cv2.imshow("Bird's Eye View", transformed_frame)
        # cv2.imshow("Lane Detection", mask)
        # cv2.imshow("Sliding Windows", window_mask)
        # cv2.imshow("Lane Highlight", overlay)
        # cv2.imshow("Original Lane", overlay_original)
        cv2.imshow("Result", result)

        return lane_offset, steering_angle
