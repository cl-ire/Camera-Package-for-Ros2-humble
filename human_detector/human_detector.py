import cv2
import os


class HumanDetector():
    def __init__(self):
        # Initialize the HumanDetector class with necessary attributes
        self.name = "HumanDetector"

        # xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'haarcascade_fullbody.xml')
        xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'haarcascade_frontalface_default.xml')

        self.full_body_cascade = cv2.CascadeClassifier(xml_path)
        self.tracker_bbox = None
        self.frame_counter = 0
        self.show_frame = False

    def locate_person(self, frame):
        values = []

        # Process the frame to detect and track humans
        self.process_frame(frame)

        # If a person is being tracked, gather information
        if self.tracker_bbox is not None:
            bbox = self.tracker_bbox
            frame_height, frame_width, _ = frame.shape
            custom_x, custom_y = self.cv_to_custom_coordinates(
                x_cv=bbox[0], y_cv=bbox[1], frame_width=frame_width, frame_height=frame_height)
            custom_center_of_person = self.cv_to_custom_coordinates(
                x_cv=bbox[0] + bbox[2] // 2, y_cv=bbox[1] + bbox[3] // 2, frame_width=frame_width, frame_height=frame_height)

            # custom_x center of person
            values.append(custom_center_of_person[0])
            # custom_y center of person
            values.append(custom_center_of_person[1])
            values.append(bbox[2])  # width of person
            values.append(bbox[3])  # height of person
            values.append(frame_width)
            values.append(frame_height)

        return values

    def process_frame(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        humans = self.full_body_cascade.detectMultiScale(
            gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Select a human to track every 5 frames
        if self.frame_counter % 5 == 0:
            # sets self.tracker_bbox
            self.select_human(frame, humans)

        # Visualize the bounding boxes of detected faces (for testing purposes)
        for (x, y, w, h) in humans:
            color = (255, 0, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        self.frame_counter += 1
        if self.show_frame:
            frame = self.draw_coordinate_system(frame)
            cv2.imshow("Video Stream", frame)

        return frame

    def select_human(self, frame, humans):
        # Select the first detected human for tracking
        if len(humans) > 0:
            selected_human = humans[0]
            x, y, w, h = selected_human
            self.tracker_bbox = (x, y, w, h)
        else:
            # No human detected, reset the tracker attributes
            self.tracker_bbox = None

    def get_percentage_of_height(self, location, frame_height):
        if frame_height > 0:
            percentage_of_height = (location[3]/frame_height*100)
            return round(percentage_of_height)
        else:
            return None

    def draw_coordinate_system(self, frame):
        height, width, _ = frame.shape

        cv2.line(frame, (0, height // 2), (width, height // 2),
                 (0, 0, 255), 2)  # Horizontal line (x-axis)
        cv2.line(frame, (width // 2, 0), (width // 2, height),
                 (0, 0, 255), 2)  # Vertical line (y-axis)

        x = width // 2
        count = 0
        while x < width:
            x += 10
            count += 10
            cv2.line(frame, (x, height // 2 - 2),
                     (x, height // 2 + 2), (0, 0, 255), 2)
            if count % 100 == 0:
                cv2.line(frame, (x, 0), (x, height), (255, 100, 0), 1)
                cv2.putText(frame, str(count), (x, height // 2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

        x = width // 2
        count = 0
        while x > 0:
            x -= 10
            count -= 10
            cv2.line(frame, (x, height // 2 - 2),
                     (x, height // 2 + 2), (0, 0, 255), 2)
            if count % 100 == 0:
                cv2.line(frame, (x, 0), (x, height), (255, 100, 0), 1)
                cv2.putText(frame, str(count), (x, height // 2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

        y = height // 2
        count = 0
        while y < height:
            y += 10
            count -= 10
            cv2.line(frame, (width // 2 - 2, y), (width // 2 + 2, y),
                     (0, 0, 255), 2)
            if count % 100 == 0:
                cv2.line(frame, (0, y), (width, y), (255, 100, 0), 1)
                cv2.putText(frame, str(count), (width // 2 + 5, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

        y = height // 2
        count = 0
        while y > 0:
            y -= 10
            count += 10
            cv2.line(frame, (width // 2 - 2, y), (width // 2 + 2, y),
                     (0, 0, 255), 2)
            if count % 100 == 0:
                cv2.line(frame, (0, y), (width, y), (255, 100, 0), 1)
                cv2.putText(frame, str(count), (width // 2 + 5, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

        return frame

    def cv_to_custom_coordinates(self, x_cv, y_cv, frame_width, frame_height):
        x_custom = x_cv - frame_width // 2
        y_custom = frame_height // 2 - y_cv
        return x_custom, y_custom

# Beispiel für die Verwendung des HumanDetector
if __name__ == "__main__":
    video_capture = cv2.VideoCapture(0)

    detector = HumanDetector