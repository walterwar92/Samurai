# BallDetection.msg — YOLO ball detection result
std_msgs/Header header
string colour           # detected ball colour: red, blue, green, yellow, etc.
string ball_type        # ball type if classified
float32 x               # bounding box top-left x (pixels)
float32 y               # bounding box top-left y (pixels)
float32 w               # bounding box width (pixels)
float32 h               # bounding box height (pixels)
float32 confidence      # detection confidence [0..1]
float32 distance_est    # estimated distance (metres), -1 if unknown
