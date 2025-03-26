
Those who need to be run with root:
```sh
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/husky_base.service /etc/systemd/system/
    sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/sensors.service /etc/systemd/system/
    # sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/multiespectral_cameras.service /etc/systemd/system/
    # sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/fisheye_cameras.service /etc/systemd/system/
```

```sh
    sudo systemctl enable husky_base.service
    sudo systemctl enable sensors.service
    # sudo systemctl disable multiespectral_cameras.service # Not used for now!
    # sudo systemctl disable fisheye_cameras.service # Not used for now!
    # sudo systemctl enable multiespectral_cameras.service
    # sudo systemctl enable fisheye_cameras.service
```

```sh
    sudo systemctl start husky_base.service
    sudo systemctl start sensors.service
    # sudo systemctl start multiespectral_cameras.service
    # sudo systemctl start fisheye_cameras.service
```


No root needed :)
```sh
    systemctl --user enable /home/administrator/eeha/test_utils/systemctl_services/conky.service
```

Start all three services
```sh
    systemctl --user start conky.service
```