# The Autonomous Weeding Garden Robot (AWGR)

[User operation video](https://drive.google.com/file/d/1jZLO9X0hrXnLbDT-zOZEFcToG8p_CPjy/view?usp=sharing)
There are a number of folders in this repo to help develop the AWGR. There are README files in each directory with further clarification of individual files. 
## RC Control 
Guide on how to setup RC mode for the robot. Scripts for microcontrollers.
## automation 
Guide for the robots autonomy. Includes information on the script that runs on startup, and the script itself. 
## drawings
.png files of all the parts that were either 3D printed or machined for the robot 
## schematics_and_drawings
Similar to the above, including all the solidworks files for each assembly. Ask your mechanical engineer. Also includes the electrical wiring diagram. 
## some_examples 
Files that we used for development and understanding of Jetson GPIO, realsense, serial communications, etc. The README in there describes how to run each example. 
## training 
Guide for training the neural network that the machine will use to discern between weeds and plants. Contains some README information from [roboflow](https://app.roboflow.com) that is helpful in creating a database for the training algorithm to use. 
