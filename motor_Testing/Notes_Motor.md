# Motor Info

### Pin Numbering
Info from seed 2024 group:\
Orin GPIO lib is broken. Instead the RPi library is used. Pinouts do not match!

Seed 25:\
Follow up on this! See if this: https://www.yahboom.net/study/Jetson-Orin-NANO



Motors use UART Serial\
View the "simplified serial" section in the documentation here
https://www.dimensionengineering.com/datasheets/Sabertooth2x12.pdf


Before sending data to uart controllers, the usb device needs to be allowed using
chmod 777 /dev/ttyTHS0

`sudo crontab -e` allows you to edit the root user crontab. This runs at startup. Adding : `@reboot chmod 777 /dev/ttyTHS0` to the root crontab allows this to be run at boot.


