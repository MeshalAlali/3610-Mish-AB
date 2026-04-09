robot_course_sim.py

what it does
- simulates the main flow from the matlab file
- followLine -> turn180 -> followLine -> turn90 -> followWall
- draws a 2d course with black tape, walls, the robot body, reflectance sensors, and ultrasonic rays
- shows phase, pose, motor duty, reflectance values, and ultrasonic distances
- has start/pause, step, reset, and speed controls
- supports --headless mode for quick testing

run it
python3 robot_course_sim.py

or headless
python3 robot_course_sim.py --headless

notes
- this is an approximation of the robot and course, not exact real-world physics
- the controller logic and phase flow are based on the matlab file, but the turns/course are idealized enough to be visual and testable on a laptop
