import numpy as np

import cv2

colors = {"blue": (255, 0, 0), "green": (0, 255, 0), "red": (0, 0, 255), "yellow": (0, 255, 255), "pink": (255, 0, 255)}


def find_best_matching_contour(contour_object, contours_scene):
    smallest_difference = 0.2
    index_best = None

    for index, contour_scene in enumerate(contours_scene):
        difference = cv2.matchShapes(contour_scene, contour_object, 1, 0.0)
        if difference < smallest_difference:
            smallest_difference = difference
            index_best = index

    return index_best, smallest_difference


def get_center_on_image(contour, image, drawing_color=None):
    M = cv2.moments(contour)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    if drawing_color is not None:
        cv2.circle(image, (cx, cy), 1, drawing_color)

    return cx, cy


def get_contour_angle_on_image(contour, image, drawing_color=None):
    x1, y1, x2, y2 = get_contour_line_on_image(contour, image)
    angle_rad = np.arctan((y2 - y1) / (x2 - x1))
    angle_deg = angle_rad * 180 / np.pi + 90

    if drawing_color is not None:
        cv2.line(image, (x1, y1), (x2, y2), drawing_color)

    return angle_deg


def get_contour_line_on_image(contour, image):
    rows, cols = image.shape[:2]
    [vx, vy, x, y] = cv2.fitLine(contour, cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)
    # First point
    x1 = 0
    y1 = int((-x * vy / vx) + y)
    # Second point
    x2 = cols - 1
    y2 = int(((cols - x) * vy / vx) + y)

    return x1, y1, x2, y2


def get_contours(image, show=False):
    # Only look at the interesting brightness values
    prepared_image = adjust_gamma(image, val_min=40, val_max=60)
    # Only pay attention to objects nearer/darker than ... Else --> 0 (ignore, is ground)
    # Delete objects further away than ...
    ret, prepared_image = cv2.threshold(prepared_image, ((55 - 40) * 255) / (60 - 40), 0, cv2.THRESH_TOZERO_INV)
    # Remove noise
    prepared_image = cv2.morphologyEx(prepared_image, cv2.MORPH_OPEN,
                                      cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    if show:
        show_image(prepared_image, "Prepared Image")

    # Prepare image edges
    # edges = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    # if show:
    #     show_image(edges, "Edges")

    # Find Contours
    contours = cv2.findContours(prepared_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Filter useful contours
    useful_contours = []
    corners = [(2, 2), (2, image.shape[0] - 3), (image.shape[1] - 3, 2), (image.shape[1] - 3, image.shape[0] - 3),
               (image.shape[1] - 3, (image.shape[0] - 3)/2)]
    for contour in contours:
        # Only select contours with more than 40 points
        if len(contour) > 50:
            in_corner = False
            # Test if Contour is in one of the corners
            for point in corners:
                if cv2.pointPolygonTest(contour, point, False) == 1:
                    in_corner = True
            if not in_corner:
                useful_contours.append(contour)

    # Sort useful contours by their number of points
    useful_contours.sort(key=len, reverse=True)

    # Show useful contours
    if show:
        image_show = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        # drawContours(image, contours, contourIdx, color, thickness)
        cv2.drawContours(image_show, useful_contours, -1, (0, 255, 255), 1)
        for corner in corners:
            cv2.circle(image_show, corner, 1, color=(255, 0, 0), thickness=1)
        show_image(image_show, "Useful Contours")
    return useful_contours


def adjust_gamma(image, val_min=60, val_max=100):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values

    table = []
    for i in range(256):
        value = ((i - val_min) * 255) / (val_max - val_min)
        if i < 30:
            value = ((50 - val_min) * 255) / (val_max - val_min)
        if value <= 0:
            value = 0
        if value > 255:
            value = 255
        table.append(value)
    table = np.array(table).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


def convert_img_msg(cv_bridge, img_msg):
    cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")
    # Convert 32fc1 to 8uc1 (Gray scale)
    image = (cv_image * 255).astype('u1')
    # helper.show_image(image, "Origin")
    image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    return image, image_rgb


def show_image_wait(image, window_name="Stream"):
    while True:
        show_image(image, window_name)
        pressed_key = cv2.waitKey(1000)
        print(pressed_key)
        if pressed_key != -1:  # Button pressed
            break


def show_image(image, window_name="Stream"):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, image.shape[1] * 4, image.shape[0] * 4)
    cv2.imshow(window_name, image)  # Show image
