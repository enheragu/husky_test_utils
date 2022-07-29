# Python Test Utils

Set of python scripts with utilities for logging, configuration and data processing.

## Dependencies

To install required dependencies you can use requirements file as follows Note that it assumes that `python3` is already installed:

```sh
    $ xargs sudo apt-get install <requirements
```

## Scripts

### gps_parser_map.py

Utilities to parse GPS topic infor stored as file and make some operations with it. 
It is mainly though to project the trajectoires and its associated error (based on covariance matrix provided by the antenna) over a given map (image).

### gui_antena_config.py

GUI to configure common Harxon ts100 Smart Antenna setup over serial port. 

The interface can be run (if the script has execution permission) as follows:
```sh
    $ ./gui_antena_config.py [-h] [-p PORT] [-b BAUDRATE]
```

Note that the port and baudrate are set as default but can be changed when executing the interface:

```sh
    optional arguments:
    -h, --help            show this help message and exit
    -p PORT, --port PORT  Port in which the antenna is connected. Takes /dev/ttyUSB0 as default.
    -b BAUDRATE, --baudrate BAUDRATE
                            Baudrate for communicating with the device. Takes 115200 as default.
```
