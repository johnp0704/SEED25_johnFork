# Arduino RC mode
 __Libraries: SoftwareSerial__
## SEED_Controller_V3_Trim.ino
Circuit: Arduino nano with an HM-10 BT module connected as such: [VCC->5V, Gnd->Gnd, TXD->6, RX->7], joystick with analog pins connected to A0 & A1, 2 potentiometers connected to A2 & A3 for trim adjustment, and 2 buttons connected to pins 3 and 4
<br/>When the arduino is turned on, the bluetooth module will connect to the SEED_Receiver bluetooth module, the SEED_Receiver arduino MUST be turned on prior to turning on the SEED_Controller arduino. 
<br/>The script will take inputs from the joystick and buttons and send them to the receiver module in the following structure over UART Serial @ 9600 baud: A0 value, A1 value, button 1 value, button 2 value. The trim value adjusts the 1st and 2nd joystick value so at rest the motors are off, this can be adjusted on the fly while the module is active with the 2 potentiometers
## SEED_Controller_V3_Receiver 1.ino
