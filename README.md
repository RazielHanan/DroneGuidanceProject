# DroneGuidanceProject
The purpose of this project is to guide a person on his way to a predetermined destination using a drone without colliding with obstacles. Here you can find all the modules of the system. in this project we used the DJI Tello Drone. but the DJITelloPy will work on any of their Drones since we used the 2.0V SDK.

# (1) SETUP:

* clone the repository:

`git clone https://github.com/RazielHanan/DroneGuidanceProject`

* install OpenCV:

`pip install opencv-python`

* install DJITelloPy:

`pip install djitellopy`

link to DJITelloPy possible functions: https://djitellopy.readthedocs.io/en/latest/tello/

# (2) How to start the system:
* locate your drone in a open space. 
* run 'Project_V1.1.py' module.
* takeoff: press 'p' until you get the desired height (8-10 meters).
* press 'l' to start the system.
* land the drone and close the programs and video: press 'q'.
> ### optional:
* you can press anytime 'r' key to stop the drone and end the tracking after objects.

# (3) Modules:

* **Project_V1.1.py** - the Main script. unite the different modules and call them by specific order.

> #### Detection and Tracking Modules:

>> [1] first of all, after receiving each new frame from the drone we will want to detect the objects in the image:

* **UserRecognition.py** - includes 'detect_user' function. Responsible for the module that deals with tracking the user in the frames received from the drone.

* **DestinationRecognition.py** - includes 'detect_target' function. Responsible for the module that deals with tracking the Destination in the frames received from the drone.

* **ObstaclesDetector.py** - includes 'detect_obstacles' function. Responsible for the module that deals with tracking the obstacles in the frames received from the drone.

> #### velocity estimination and predition

>> [2] then, we want to get information about this objects:

* **ObstacleState.py** - here we define class that describes the User/Destination/Obstacle. after declaring a variable of this class we should add the new positions of the object we tracking every time we detecting it on new frame. this class will automatically calculate the velocity of the object and it will be available at self.vx and self.vy attributes for both axises. in addition, this class automatically calculates the future positions of the object in the next 3 seconds by assuming a constant speed.

> #### Navigation and Guidance Modules:

* **Path.py** - 


