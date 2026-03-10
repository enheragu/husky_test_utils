# Systemd Services

Service files for managing the full Husky software stack via `systemctl`. All services run as root except Conky.

## Installation

### Symlink service files (as root)

```sh
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/roscore.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/husky_base.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/sensors.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/localization.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/ptp_cameras_lidar.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/ptp_phc2sys.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/husky_web_manager.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/multiespectral_cameras.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/fisheye_cameras.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/multiespectral_gui.service /etc/systemd/system/
sudo ln -s /home/administrator/eeha/test_utils/systemctl_services/fisheye_gui.service /etc/systemd/system/
```

### Enable on boot

```sh
sudo systemctl enable roscore husky_base sensors localization conky
sudo systemctl enable ptp_cameras_lidar ptp_phc2sys husky_web_manager
# Camera services are started manually (not enabled on boot):
# sudo systemctl enable multiespectral_cameras fisheye_cameras
```

### Conky (user service, no root)

```sh
systemctl --user enable /home/administrator/eeha/test_utils/systemctl_services/conky.service
systemctl --user start conky.service
```

## Boot Order

```
ptp_cameras_lidar → ptp_phc2sys → roscore → husky_base
                                          → sensors → localization
                                          → husky_web_manager → multiespectral_gui
                                                               → fisheye_gui
                                          → conky
```

Camera services (`multiespectral_cameras`, `fisheye_cameras`) are started on demand via the shell functions or web dashboard.

## Useful Commands

```sh
# Check service status
sudo systemctl status sensors.service

# View logs
journalctl -u sensors.service --no-pager -n 50

# Clear journal
sudo journalctl --rotate && sudo journalctl --vacuum-time=1s
```
