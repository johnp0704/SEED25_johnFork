# Autonomy Script 
The autonomy of the robot relies on a python script that uses the neural network weights from the training above. The script uses a realsense camera library to identify the weeds and determine their distance from the robot. The Jetson navigates to the closest plant, using serial communication to the sabertooth 2x12 motor controller , then outputs a digital high, to signal to the Arduino nano to actuate and turn on the drill. The script also accounts for memory leaks by including a shutdown sequence that can be triggered by pressing a button connected to the outside of the robot. This cleans up and closes GPIO sessions, realsense session, and openCV sessions. This prevents these sessions from remaining open on shutdown, which would leave the memory allocated for them with no way to re-enter and close those sessions. 

The python script described above is compiled and run using a command line command that is coded into Jetson’s `crontab` formatting so that it runs when the machine boots up, with no necessary input from the user. 
use `crontab -e` to edit the startup script, you can disable the script running on startup by commenting out the command that starts with `sudo ...` with a `#` at the beginning.

## MotorAutonomy.py 
uses the realsense library, the neural network trained in the training directory, serial communication with motors, depth calculations, and GPIO digital outputs to communicate with an external microcontroller to operate the entirety of the robot. 

## shell script 
Shell scripts (.sh) are essentially text files with lists of unix commands to execute at the command line level. Making a shell script executable with a `chmod +x` command allows the script to be run on startup using the machines `crontab`.
You can also comment out the code here to disable starting the script on boot, or edit what will run on startup. 
