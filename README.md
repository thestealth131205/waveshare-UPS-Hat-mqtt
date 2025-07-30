# waveshare-UPS-Hat-mqtt
A script for 2 way mqtt of the hat tasmota compatible


INSTALL

sudo raspi-config
Select Interfacing Options -> I2C ->yes to start the i2C kernel driver

sudo reboot


sudo apt-get install python3-smbus 

pip install paho-mqtt

sudo nano /etc/systemd/system/ups_hat.service



[Unit]
Description=UPS HAT MQTT Service
After=network.target

[Service]
Type=simple
User=thestealth
WorkingDirectory=/home/userdir/UPS_HAT
ExecStart=/usr/bin/python3 /home/userdir/UPS_HAT/ups-hat-rpi.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target



enjoy

