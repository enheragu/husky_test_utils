
Those who need to be run with root:
```sh
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/roscore.service /etc/systemd/system/
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/husky_base.service /etc/systemd/system/
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/sensors.service /etc/systemd/system/
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/conky.service /etc/systemd/system/
    # sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/multiespectral_cameras.service /etc/systemd/system/
    # sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/fisheye_cameras.service /etc/systemd/system/
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/ptp_cameras_lidar.service /etc/systemd/system/
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/ptp_phc2sys.service /etc/systemd/system/
```

```sh
    sudo systemctl enable roscore.service
    sudo systemctl enable husky_base.service
    sudo systemctl enable sensors.service
    sudo systemctl enable conky.service
    # sudo systemctl disable multiespectral_cameras.service # Not used for now!
    # sudo systemctl disable fisheye_cameras.service # Not used for now!
    # sudo systemctl enable multiespectral_cameras.service
    # sudo systemctl enable fisheye_cameras.service
    sudo systemctl enable ptp_cameras_lidar.service
    sudo systemctl enable ptp_phc2sys.service
```

```sh
    sudo systemctl start roscore.service
    sudo systemctl start husky_base.service
    sudo systemctl start sensors.service
    sudo systemctl start conky.service
    # sudo systemctl start multiespectral_cameras.service
    # sudo systemctl start fisheye_cameras.service
    sudo systemctl start ptp_cameras_lidar.service
    sudo systemctl start ptp_phc2sys.service
```


No root needed :)
```sh
    systemctl --user enable /home/administrator/eeha/test_utils/systemctl_services/conky.service
```

Start all three services
```sh
    systemctl --user start conky.service
```



clear journalctl:
```sh
sudo journalctl --rotate
sudo journalctl --vacuum-time=1s
```
