# DroneGuidanceProject
The purpose of this project is to guide a person on his way to a predetermined destination using a drone without colliding with obstacles. In this project we used the DJI Tello Drone, but the DJITelloPy (the library used to implement our design) will work on any of the DJI Drones since we used the 2.0V SDK.
Here you can find all the modules of the system.

# (1) SETUP:
* open terminal

* install git:

`pip install python-git`

* clone the repository:

`git clone https://github.com/RazielHanan/DroneGuidanceProject`

* install OpenCV:

`pip install opencv-python`

* install DJITelloPy:

`pip install djitellopy`

link to DJITelloPy functions: https://djitellopy.readthedocs.io/en/latest/tello/

# (2) How to start the system:
* locate your drone in an open space. 
* run 'Project_V1.1.py' module.
* takeoff: press 'p' until you get the desired height (8-10 meters).
* press 'l' to start the system.
* press 'q' land the drone and close the programs and video.
> ### optional:
* you can press the 'r' key anytime to stop the drone and end the tracking after objects.

# (3) Modules:

* **Project_V1.1.py** - the Main script. Unites the different modules and call them by specific order.

> #### **Detection and Tracking Modules:**

>> [1] first of all, after receiving each new frame from the drone we will want to detect the objects in the image (the user, destination and obstacles):

* **UserRecognition.py** - includes 'detect_user' function. Responsible for the module that deals with tracking the user in the frames received from the drone.

* **DestinationRecognition.py** - includes 'detect_target' function. Responsible for the module that deals with tracking the Destination in the frames received from the drone.

* **ObstaclesDetector.py** - includes 'detect_obstacles' function. Responsible for the module that deals with tracking the obstacles in the frames received from the drone.

In all of those modules, there are automatic data prints of the tracking performance and positions of objects to the log files.

> #### **Velocity Estimation and Prediction Modules**

>> [2] then, we want to get information about these objects:

* **velocity_estimation.py** - gets the positional history of objects and matches a linear function to those positions. we assume a constant velocity so the slope of said linear function is our estimated velocity of the object.

* **ObstacleState.py** - here we define a class that describes the User/Destination/Obstacles. after declaring an instance of this class we will add the new positions of the object we are tracking every time we detect it in a new frame. this class will automatically calculate the velocity of the object and it will be available at self.vx and self.vy attributes for both axises. in addition, this class automatically estimates the future positions of the object in the next 3 seconds by assuming a constant velocity.

> #### **Navigation and Guidance Modules:**

>> [3] now we have the necessary information to calculate an appropriate path from the user to the destination:

* **Path.py** - includes 'get_best_angle' function.

inputs - user position, destination position, obstacles positions and velocities.

functionality - here we will calculate the optimal angle for the next step (in the path) only.

* **FullPath.py** -includes 'get_full_path' function.

inputs - user position, destination position, obstacles positions and velocities.

functionality - here we will call 'get_best_angle' by iterations. every iteration we will change the starting position of the user 1 step towards the best angle from the last iteration (starting from the user’s current position and ending in the destination’s position). in this way we will get a complete path from the user’s current position to the destination. in order to check the performances of the algorithm, we will print the calculation time of each iteration to the log file.

> #### **Drone Control Loop Modules:**

>> [4] Also, we want the drone to follow the user, in order to keep it in it’s field of view. We implement this by closing a velocity control loop between the drone and the user (which will also close the positional error). It will be using the user’s velocity and position:

* **velocity_controller.py** - includes 'velocity_cmd' attribute of the Controller class which calculates the desired velocity of the drone in order to follow the user using PD controller.

* **UserTracker.py** - here we calculate the error between the current position and the desired position of the user in comparison to the drone. we then send the appropriate velocity command to the drone after calling 'velocity_cmd' attribute and printing the command to the log file for debugging purposes.

> #### **saving the frames we recieve from the drone:**

* **defines.py** - define constants in order to save the frames in a specific directory.

* **write.py** - class for comfortable image saving in the main loop.

# (4) Products:

* **5 videos of the system’s operation in different environmental conditions** - 

link to Google Drive with the videos - https://drive.google.com/drive/folders/1DrBqHkqisXKfT6CVTljfCKJ61MzeInZ8?usp=sharing

**vid_res_1.mp4** - A single obstacle, the system offers a possible path at each stage. Even when the user does not follow the suggested path, the system corrects the route at each step.

**vid_res_2.mp4** - 3 obstacles cross the suggested path back and forth between the user and the destination. The system deals with these disturbances using velocity and position estimators.

**vid_res_3.mp4** - Several obstacles that interfere with the path, a circular movement of the obstacles to disrupt the velocity estimators and prediction values. Dealing with changing the destination location (the system must regularly update the path to avoid obstacles).

**vid_res_4.mp4** - The system deals with a random scenario - many obstacles, with no set pattern.

**vid_res_5.mp4** - Illustrates a possible scenario where the destination’s position changes, the system must respond in real time.

Each of these product videos has documentation files as described in the explanation of the various modules.


