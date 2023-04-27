
in :
cd /home/pi/robot_rpi_2023/services
creat  a toto.service file

then to initiate them:
cp toto.service /opt/services
sudo systemctl enable /opt/services/toto.service

then to interact:
sudo systemctl [opt] toto.service
[opt]  = 
        status
        start
        restart
        stop
(see man for further infos)
        
        
file example : 
[Unit]
Description= toto totoing
After=toto2.service
Wants=toto3.service

[Service]
Type=simple
 
User=pi
Group=pi

SupplementaryGroups=adm dialout cdrom sudo audio video plugdev games users input netdev gpio i2c spi
 
ExecStart= toto_cmd.extension
 
Restart=on-failure
RestartSec=2 
# Configures the time to wait before service is stopped forcefully.
TimeoutStopSec=300
 
[Install]
WantedBy=multi-user.target
