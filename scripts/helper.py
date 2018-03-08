import numpy as np

import cv2


def get_object_rotation_in_scene(image_object, image_scene, show=False):
    contours_scene = find_contours(image_scene, show)
    contour_object = find_contours(image_object, show)[0]

    angle_object = get_contour_angle_on_image(contour_object, image_object, False)

    matching_ranking = {}
    for index in range(len(contours_scene)):
        contour_scene = contours_scene[index]
        if len(contour_scene) > 200:
            matching = cv2.matchShapes(contour_scene, contour_object, 1, 0.0)
            if matching < 0.7:
                matching_ranking[index] = matching

    # Get contour with best matching
    index = min(matching_ranking, key=matching_ranking.get)
    contour_scene = contours_scene[index]

    # Show best result
    print("Contour Length:", len(contour_scene))
    print("Matching:", matching_ranking[index])
    cv2.drawContours(image_scene, contours_scene, index, (0, 255, 255), 3)
    angle_in_scene = get_contour_angle_on_image(contour_scene, image_scene, True)
    print("Angle difference", angle_in_scene - angle_object)


def get_contour_angle_on_image(contour, image, show_line=False):
    x1, y1, x2, y2 = get_contour_line_on_image(contour, image)
    angle_rad = np.arctan((y2 - y1) / (x2 - x1))
    angle_deg = angle_rad * 180 / np.pi + 90

    if show_line:
        draw_line(image, x1, y1, x2, y2)

    return angle_deg


def draw_line(image, x1, y1, x2, y2):
    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    show_image_wait(image)


def get_contour_line_on_image(contour, image):
    rows, cols = image.shape[:2]
    [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
    # First point
    x1 = 0
    y1 = int((-x * vy / vx) + y)
    # Second point
    x2 = cols - 1
    y2 = int(((cols - x) * vy / vx) + y)

    return x1, y1, x2, y2


def find_contours(image, show=False):
    # Only look at the interesting brightness values
    prepared_image = adjust_gamma(image, val_min=40, val_max=60)
    # Only pay attention to objects nearer/darker than ... Else --> 0 (ignore, is ground)
    # Delete objects further away than ...
    ret, prepared_image = cv2.threshold(prepared_image, ((54 - 40) * 255) / (60 - 40), 0, cv2.THRESH_TOZERO_INV)
    # Remove noise
    prepared_image = cv2.morphologyEx(prepared_image, cv2.MORPH_OPEN,
                                      cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    show_image(prepared_image, "Prepared Image")

    # Prepare image edges
    # edges = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    # if show:
    #     show_image(edges, "Edges")

    # Find Contours
    contours = cv2.findContours(prepared_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    # Sort Contours by their number of points
    contours.sort(key=len, reverse=True)

    # Show found contours
    if show:
        image_show = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        # drawContours(image, contours, contourIdx, color, thickness)
        cv2.drawContours(image_show, contours, -1, (0, 255, 255), 3)
        show_image(image_show, "Found Contours")
    return contours


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


if __name__ == "__main__":
    image_scene_pico = cv2.imread('power1.png')
    image_power_bank = cv2.imread('power_origin2.png')
    # show_image(image_scene_pico)

    get_object_rotation_in_scene(image_power_bank, image_scene_pico, show=False)
