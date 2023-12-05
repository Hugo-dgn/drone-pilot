# ROS Package Documentation

## Camera Feedback

The camera feedback is handled by the `camera_feedback` package. This package provides an OpenCV window that displays the camera feed from the drone's front camera.

The camera feed is provided in grayscale to maximize performance.

The windows detected by the `window_detector` package are displayed with white rectangles. It's important to note that the rectangle assumed to be the target window is drawn in black. Furthermore, the ID of each potential window is located in the top-right corner of the respective window.

## Droneload Messages

All custom ROS messages are defined in the `droneload_msgs` package. Here is the definition of those messages:

- `Point2D`: Defines a point in two dimensions. This message contains `Point2D.x` and `Point2D.y`, representing the position in the `x` and `y` dimensions.

- `Path`: Defines a path in three dimensions. This is a list of `geometry_msgs/Point` messages accessible with `Path.points`.

- `Rectangle2D`: Defines a rectangle in two dimensions. This includes four points defined with `Point2D` and accessible via `Rectangle2D.p1`, `Rectangle2D.p2`, `Rectangle2D.p3`, and `Rectangle2D.p4`. Additionally, the ID of this rectangle is stored in `Rectangle2D.id`.

- `Rectangles2D`: Defines a list of `Rectangle2D` accessible with `Rectangles2D.rectangles`.

- `Rectangles3D`: Defines a rectangle in three dimensions. This includes four points defined with `geometry_msgs/Point` and accessible via `Rectangles3D.p1`, `Rectangles3D.p2`, `Rectangles3D.p3`, and `Rectangles3D.p4`. Additionally, the ID of this rectangle is stored in `Rectangles3D.id`.

## Image Preprocessing

The preprocessing of the camera feed is done in the `image_preprocess` package. This package converts the images to grayscale and undistorts them. The processed images are then accessible via the `pub_front_camera` ROS topic, which is defined in the parameter server.

## Parameter Server

The parameter server is set up in the `parameters` package. This defines parameters accessible by all other ROS packages. Here is a list of those parameters:

- `drone_state`: Number of the current state in the state machine
- `main_loop_rate`: Rate of the main loop (in Hz) in the `pilot` package
- `window_id`: `ID` of the rectangle assumed to be the window
- `pub_drone_state`: Path to the ROS topic to publish the drone state
- `pub_drone_local_pose_set_point`: Path to the ROS topic used to give setpoint to PX4
- `pub_drone_local_pose`: Path to the ROS topic used to track the local position of the drone
- `service_drone_arming`: Path to the arming service of the drone
- `service_drone_set_mode`: Path to the set mode service of the drone
- `pub_front_camera`: Path to the ROS topic which provides the processed front camera images
- `pub_main_rectangle`: Path to the ROS topic which contains the rectangle assumed to be the window
- `pub_all_rectangle`: Path to the ROS topic which contains all the rectangles detected on the front camera image
- `pub_window_path`: Path to the ROS topic which contains the path to follow to go through the window
- `pub_rectangle_window`: Path to the ROS package which contains the 3D rectangle of the window
- `pub_front_camera_image_raw`: Path to the ROS topic which contains the raw image of the front camera
- `path_front_camera_calibration`: Path to the `yaml` file which defines the calibration parameters of the front camera
- `path_window_points`: Number of points used in the path for the window
- `reposition_threshold`: Distance after which the drone repositions to the closest point on the path
- `window_min_dist_next_point`: Distance below which the navigation algorithm of the drone changes the setpoint.
- `front_rect_fit`: Value of the `fit` parameter for the rectangle detection.
- `front_rect_tol`: Value of the `tol` parameter for tracking rectangle.
- `front_rect_max_lifetime`: Maximum lifetime of a rectangle after the last detection
- `front_rect_min_fit_success`: Minimum detection success to define a rectangle as the main
- `front_image_size`: Size of the image given by the front camera

## pilot

The package `pilot` is the main package of this project. It put together all the compution provided by the other and performe the navigation of the drone.