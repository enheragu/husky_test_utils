

```sh
    systemctl --user enable /home/administrator/eeha/test_utils/systemctl_services/husky_base.service
    systemctl --user enable /home/administrator/eeha/test_utils/systemctl_services/sensors.service
    systemctl --user enable /home/administrator/eeha/test_utils/systemctl_services/conky.service
```

Start all three services
```sh
    systemctl --user start husky_base.service
    systemctl --user start sensors.service
    systemctl --user start conky.service
```