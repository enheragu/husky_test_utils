# Python Test Utils

Set of python scripts with utilities for logging, configuration and data processing

## Dependencies

To install required dependencies you can use requirements file as follows Note that it assumes that python3 is already `installed`:

```sh
    $ xargs sudo apt-get install <requirements
```

## Scripts

### gps_parser_map.py

Utilities to parse GPS topic infor stored as file and make some operations with it. 
It is mainly though to project the trajectoires and its associated error (based on covariance matrix provided by the antenna) over a given map (image).

### gui_antena_config.py

GUI to configure common Harxon ts100 Smart Antenna setup over serial port. 

