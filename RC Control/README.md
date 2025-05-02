# Arduino RC mode
 __Libraries: SoftwareSerial__
## SEED_Controller_V3_Trim.ino
Circuit: Arduino nano with an HM-10 BT module connected as such (HM-10 -> Arduino): [VCC->5V, Gnd->Gnd, TXD->6, RX->7], joystick with analog pins connected to A0 & A1, 2 potentiometers connected to A2 & A3 for trim adjustment, and 2 buttons connected to pins 3 and 4
<br/>When the arduino is turned on, the bluetooth module will connect to the SEED_Receiver bluetooth module, the SEED_Receiver arduino MUST be turned on prior to turning on the SEED_Controller arduino. 
<br/>The script will take inputs from the joystick and buttons and send them to the receiver module in the following structure over UART Serial @ 9600 baud: A0 value, A1 value, button 1 value, button 2 value. The trim value adjusts the 1st and 2nd joystick value so at rest the motors are off, this can be adjusted on the fly while the module is active with the 2 potentiometers
## SEED_Controller_V3_Receiver_Serial.ino
Circuit: Arduino nano with an HM-10 BT module connected as such: [VCC->5V, Gnd->Gnd, TXD->6, RX->7], 2 H-Bridges (one for the actuator and one for the drill motor) connected as such: (See [this](https://www.hessmer.org/blog/2013/12/28/ibt-2-h-bridge-with-arduino/) for more info)
<br/> Actuator (H-Bridge -> Arduino):
<br/>- R_EN, L_EN, R_IS, L_IS, VCC -> 5V
<br/>- Gnd -> Gnd
<br/>- RPWM -> 4
<br/>- LPWM -> 8
<br/> Drill Motor (H-Bridge -> Arduino):
<br/>- R_EN, L_EN, R_IS, L_IS, VCC -> 5V
<br/>- Gnd -> Gnd
<br/>- RPWM -> 2
<br/>- LPWM -> 9
<br/>The Sabertooth 2x12 motor controller is connected to the motors (M1 for left side and M2 for right side) and to the arduino as such (MC -> Arduino):
<br/>- 5V -> Vin
<br/>- 0V -> Gnd
<br/>- S1 -> 1 (TX1)
<br/>- S2 -> Floating
<br/>The script takes the data sent from the controller in the [A0 value, A1 value, button 1 value, button 2 value] format and formats the motor commands from 0 -> 1023 sent range into the 1 -> 127 and 128 -> 255 ranges that the Sabertooth 2x12 motor controller accepts in simplified serial mode, see this [datasheet](https://www.dimensionengineering.com/datasheets/Sabertooth2x12.pdf) for more info. It also takes the button 1 and 2 values and maps them to moving the actuator up when the actuator button is pressed (value drops from 1 to 0) and spinning the drill motor when the motor button is pressed (value drops from 1 to 0).
## SEED_Controller_V4_Receiver_Jetson.ino
This script has all of the functionality of the SEED_Controller_V3_Receiver_Serial.ino script as well as an additional digital pin that reads a signal coming from the Jetson. When pin 10 is pulled low from the Jetson, the drill motor will start spinning and the actuator will raise for a specified duration (so it does not over-extend). The motor will continue spinning when the actuator is raised to this specified length until the Arduino receives a logic HIGH signal to pin 10. Once this occurs, the motor will stop and the actuator will lower to its fully-reteacted state. While the arduino has a LOW signal fed to pin 10, all other functions of the script are halted until the HIGH signal is received.

