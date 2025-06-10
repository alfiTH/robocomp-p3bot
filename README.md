# robocomp-p3bot

## Dependencies
```bash
sudo apt-get install qdbus
vcs import $ROBOCOMP/components < p3bot.repos --recursive
```

## udev rules
```bash
sudo cp etc/10-p3bot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## vnc server
```bash
sudo apt install x11vnc -y

sudo x11vnc -storepasswd /etc/x11vnc.pass
sudo chmod 600 /etc/x11vnc.pass
sudo chown robolab:robolab /etc/x11vnc.pass

sudo cp etc/services/x11vnc.service /etc/systemd/system/x11vnc.service

sudo systemctl daemon-reload
sudo systemctl enable x11vnc.service
sudo systemctl start x11vnc.service
```
