import cv2
import numpy as np

def find_circles(image, params):
    blur_amount = 5

    x_min   = params["x_min"]
    x_max   = params["x_max"]
    y_min   = params["y_min"]
    y_max   = params["y_max"]

    search_area = [x_min, y_min, x_max, y_max]

    blurred_image = cv2.blur(image, (blur_amount, blur_amount))

    if search_area is None: 
        search_area = [0.0, 0.0, 1.0, 1.0]

    search_area_pixels = convert_rect_perc_to_pixels(search_area, image)

    hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

    min_threshold = (params["h_min"], params["s_min"], params["v_min"])
    max_threshold = (params["h_max"], params["s_max"], params["v_max"])

    hsv_image = cv2.inRange(hsv_image, min_threshold, max_threshold)

    hsv_image = cv2.dilate(hsv_image, None, iterations=2)
    hsv_image = cv2.erode(hsv_image, None, iterations=2)

    masked_image = cv2.bitwise_and(image, image, mask=hsv_image)

    masked_image = apply_search_window(masked_image, search_area)

    inverted_image = 255 - hsv_image

    blob_params = cv2.SimpleBlobDetector_Params()
    blob_params.minThreshold = 0    
    blob_params.maxThreshold = 100
    blob_params.filterByArea = True
    blob_params.minArea = 30
    blob_params.maxArea = 20000
    blob_params.filterByCircularity = True
    blob_params.minCircularity = 0.1
    blob_params.filterByConvexity = True
    blob_params.minConvexity = 0.5
    blob_params.filterByInertia = True
    blob_params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(blob_params)

    keypoints = detector.detect(inverted_image)     #points used in "drawKeypoints function to draw blob circles"

    min_size_px = params['sz_min'] * inverted_image.shape[1] / 100.0
    max_size_px = params['sz_max'] * inverted_image.shape[1] / 100.0
    keypoints = [k for k in keypoints if k.size > min_size_px and k.size < max_size_px] #keeps the blobs which are only in the range 

    output_image = cv2.drawKeypoints(image, keypoints, None, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    output_image = draw_window(output_image, search_area_pixels)

    tuning_image = cv2.drawKeypoints(masked_image, keypoints, None, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    tuning_image = draw_window(tuning_image, search_area_pixels)

    normalized_keypoints = [normalize_keypoint(inverted_image, k) for k in keypoints]

    return normalized_keypoints, output_image, tuning_image


def apply_search_window(image, window=[0.0, 0.0, 1.0, 1.0]):
    rows, cols = image.shape[:2]

    x_min_px = int(cols * window[0] / 100)
    y_min_px = int(rows * window[1] / 100)
    x_max_px = int(cols * window[2] / 100)
    y_max_px = int(rows * window[3] / 100)

    mask = np.zeros(image.shape, np.uint8)

    mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]

    return mask


def draw_window(image, rect_pixels, color=(255, 0, 0), thickness=5):
    return cv2.rectangle(image, (rect_pixels[0], rect_pixels[1]), (rect_pixels[2], rect_pixels[3]), color, thickness)

def convert_rect_perc_to_pixels(rect_perc, image):
    rows, cols = image.shape[:2]
    scale = [cols, rows, cols, rows]

    return [int(a * b / 100) for a, b in zip(rect_perc, scale)]


def normalize_keypoint(cv_image, kp):
    rows, cols = cv_image.shape[:2]
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    x = (kp.pt[0] - center_x) / center_x
    y = (kp.pt[1] - center_y) / center_y	
    return cv2.KeyPoint(x, y, kp.size / cols)


def create_tuning_window(initial_values):
        cv2.namedWindow("Tuning", 0)
        cv2.createTrackbar("x_min", "Tuning", initial_values['x_min'], 100, dummy) #createTrackbar(name_of_bar,window_to_place,starting_value,max_value,return_value)
        cv2.createTrackbar("x_max", "Tuning", initial_values['x_max'], 100, dummy)
        cv2.createTrackbar("y_min", "Tuning", initial_values['y_min'], 100, dummy)
        cv2.createTrackbar("y_max", "Tuning", initial_values['y_max'], 100, dummy)
        cv2.createTrackbar("h_min", "Tuning", initial_values['h_min'], 180, dummy)
        cv2.createTrackbar("h_max", "Tuning", initial_values['h_max'], 180, dummy)
        cv2.createTrackbar("s_min", "Tuning", initial_values['s_min'], 255, dummy)
        cv2.createTrackbar("s_max", "Tuning", initial_values['s_max'], 255, dummy)
        cv2.createTrackbar("v_min", "Tuning", initial_values['v_min'], 255, dummy)
        cv2.createTrackbar("v_max", "Tuning", initial_values['v_max'], 255, dummy)
        cv2.createTrackbar("sz_min", "Tuning", initial_values['sz_min'], 100, dummy)
        cv2.createTrackbar("sz_max", "Tuning", initial_values['sz_max'], 100, dummy)

def get_tuning_params():
    x_min = cv2.getTrackbarPos("x_min", "Tuning")       #cv2.getTrackbarPos(name_of_bar, name_of_window)
    x_max = cv2.getTrackbarPos("x_max", "Tuning")
    y_min = cv2.getTrackbarPos("y_min", "Tuning")
    y_max = cv2.getTrackbarPos("y_max", "Tuning")
    h_min = cv2.getTrackbarPos("h_min", "Tuning")
    h_max = cv2.getTrackbarPos("h_max", "Tuning")
    s_min = cv2.getTrackbarPos("s_min", "Tuning")
    s_max = cv2.getTrackbarPos("s_max", "Tuning")
    v_min = cv2.getTrackbarPos("v_min", "Tuning")
    v_max = cv2.getTrackbarPos("v_max", "Tuning")
    sz_min = cv2.getTrackbarPos("sz_min", "Tuning")
    sz_max = cv2.getTrackbarPos("sz_max", "Tuning")

    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "h_min": h_min,
        "h_max": h_max,
        "s_min": s_min,
        "s_max": s_max,
        "v_min": v_min,
        "v_max": v_max,
        "sz_min": sz_min,
        "sz_max": sz_max
    }

def wait_on_gui():
    cv2.waitKey(2)

def dummy(x):
    pass


# cam = cv2.VideoCapture(2)

# tuning_params = {
#     'x_min': 0,
#     'x_max': 100,
#     'y_min': 42,
#     'y_max': 100,
#     'h_min': 100,
#     'h_max': 134,
#     's_min': 23,
#     's_max': 255,
#     'v_min': 0,
#     'v_max': 255,
#     'sz_min': 1,
#     'sz_max': 30
# }

# while True:
#     imTrue, frame = cam.read()
#     keypoints_norm, out_image, tuning_image = find_circles(frame, tuning_params)
    

#     cv2.imshow('Camera', out_image)
#     cv2.imshow('tuning', tuning_image)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# print(keypoints_norm)
# cam.release()
# cv2.destroyAllWindows()
