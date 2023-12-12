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

## Pilot

The `pilot` package serves as the primary package for this project, encompassing all computations provided by others and orchestrating the navigation of the drone.

The main loop of the ROS project is located in the `main.py` file. The rate of the main loop is defined by the parameter `/droneload/parameters/main_loop_rate`. Within this loop, the flight state (in the finite state machine) is checked, and transitions are performed if necessary. Currently, the possible states are:
- 0: Position mode flight
- 1: Offboard mode flight to navigate through windows

In the `drone.py` file, the definition of the Python class representing the drone can be found. The chosen design for implementing this class is as follows:
- Every parameter related to the drone, such as its flight state or position, is an attribute of the class.
- The ROS callback functions that change the attributes of the class are methods of the class.
- Minimize the use of `while` loops in the methods of the class to ensure that security and state flight mode checks in the main loop are still performed. If something must be called periodically (like the navigation system in offboard mode), call it in the main loop rather than creating another loop.

Therefore, the majority of the code in the main loop should revolve around the `drone` class.

## Window Detector

The `window_detector` package utilizes the image from the front camera to detect rectangles that might represent windows and publishes them on the `/droneload/parameters/pub_all_rectangle` topic. The rectangle assumed to be the actual window is published on the `/droneload/parameters/pub_main_rectangle` topic.

This package primarily relies on the `droneload` package, which is developed separately from this project and can be found at: [https://github.com/Hugo-dgn/droneload](https://github.com/Hugo-dgn/droneload). For further documentation, please refer to the repository.

## Window Path Finder

The `window_path_finder` package utilizes the `main rectangle` detected and published by the `window_detector` to compute the optimal path for navigating through the identified window. It then publishes the found path to the `/droneload/parameters/path_window_points` topic.

This package primarily depends on the `droneload` package, which is developed separately from this project and can be found at: [https://github.com/Hugo-dgn/droneload](https://github.com/Hugo-dgn/droneload). For additional documentation, please consult the repository.

## Window Position Feedback

The `window_pos_feedback` package generates a `3D matplotlib plot` depicting the position of the drone in space and the detected object. Currently, the plot displays the positions of the `drone` and the `window`, as well as the calculated `path` for navigating through the window.