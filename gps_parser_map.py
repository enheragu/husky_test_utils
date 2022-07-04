#!/usr/bin/env python3
# encoding: utf-8


""" This file parses tests with GPS matching YAML file with base/rover 
    position extracted from google maps and checks against ros topic
    file saved with testN_rover/base.log
"""

import yaml
import re
from pathlib import Path

from math import radians, cos, sin, atan2, sqrt, pi, degrees

from yaml.loader import SafeLoader

# Open the file and load the file
def parseYAMLConfig(filename_in):
    """
        Parses yaml file an returns data
    """
    print ("[INFO] [parseYAMLConfig] - Parsing file " + str(filename_in))
    with open(filename_in) as file:
        data = yaml.load(file, Loader=SafeLoader)
        return data

def parseROSTopicLog(filename_in):
    """
        Parses ros topic output saved as file. The information is about the GPS data published
        in /fix topic.
        Returns an array of dictionaries with the data stored

        :param filename_in: Path of the file to process
        :return: Returns array of dictionaries with the information encoded
    """

    print ("[INFO] [parseROSTopicLog] - Parsing file " + str(filename_in))

    """ The following regex pattern searchs in rostopic fix dumped file for the following data
        · match[0] = status
        · match[1] = latitude
        · match[2] = longitude
        · match[3] = altitude
        · match[4] = position_covariance[0,0] # diagonal of the matrix is parsed
        · match[5] = position_covariance[1,1]
        · match[6] = position_covariance[2,2]
        · match[7] = position_covariance_type
    """
    pattern_reg_str = "  status: ([0-9-]+)\n  service: [0-9-.]+\nlatitude: ([0-9-.]+)\nlongitude: ([0-9-.]+)\naltitude: ([0-9-.]+)\nposition_covariance: \[([0-9-.]+), [0-9-.]+, [0-9-.]+, [0-9-.]+, ([0-9-.]+), [0-9-.]+, [0-9-.]+, [0-9-.]+, ([0-9-.]+)\]\nposition_covariance_type: ([0-9-])"
    

    data = []
    with open(filename_in) as file:
        file = file.read()
        match_list = re.findall(pattern_reg_str,file)
        for index, match in enumerate(match_list):
            curr_data = {}
            curr_data["status"] = float(match[0])
            curr_data["position"] = [float(match[1]), float(match[2]), float(match[3])]
            curr_data["position_covariance"] = [float(match[4]), float(match[5]), float(match[6])]
            curr_data["position_covariance_type"] = float(match[7])

            data.append(curr_data)
    # print("[INFO] [parseROSTopicLog] - Data parsed is: " + str(data))
    return data

def haversine(lon1_in, lat1_in, lon2_in, lat2_in):
    """
        Calculate the great circle distance in meters between two points 
        on the earth (specified in decimal degrees)

        :param lon1_in: longitude of point 1
        :param lat1_in: latitude of point 1
        :param lon2_in: longitude of point 2
        :param lat2_in: latitude of point 2

        :return: Returns float distance in meters between the two points
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1_in, lat1_in, lon2_in, lat2_in])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a),sqrt(1-a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    return (c * r)*1000


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import EllipseCollection

def projectOverMapImage(filename_in, data_in):
    """
        Projects data provided over map iamge provided. Note that image has to have a configuration file
        with the pixel-GPS correspondance.
        :param filename_in: - Path of the image to process.
        :param data_in: - Data array of 2d array with gps coordinates to project over the image

        :return: Returns nothing
    """

    # Parse configuration of points in image
    data = parseYAMLConfig(filename_in.replace("jpg", "yml"))

    # print(data)
    # gps_pixel = (np.array(data[0]["gps"]) - np.array(data[1]["gps"])) / (np.array(data[0]["pixel"]) - np.array(data[1]["pixel"]))
    # print(gps_pixel)


    def get_bearing(lon1_in, lat1_in, lon2_in, lat2_in):
        lon1, lat1, lon2, lat2 = map(radians, [lon1_in, lat1_in, lon2_in, lat2_in])
        dLon = (lon2 - lon1)
        x = cos(lat2) * sin(dLon)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
        bearing = atan2(x,y)
        bearing = degrees(bearing)

        return bearing


    def getCurrentPixelAxis(upper_left_in, lower_right_in, current_in, direction_in = "X"):
        """
            Based on two point references in a map image (referenced points with matched gps and pixel coordinates)
            returns the corresponding pixel coordinate in the given axis (X or Y) of the current GPS coordinates.

            :param upper_left_in: - Upper left corner. Its a dictionary with the configuration of the map
                                    provides gps coordinates and pixel coordinates of a point. Should be
                                    as close as possible to the upper left corner of the image.
            :param lower_right_in: - Lower right corner. Analog to the upper left bot in the bottom right 
                                    corner of the image
            :param current_in: - Is an array of GPS coordinates, both lattitude and longitude of the current
                                to check.
            :param direction_in: - Direction in which the pixel coordinate is requested (X or Y axis)

            :return: Returns value of pixel
        """
        direction_index = {'X': 0, 'Y':1}
        direction_func = {'X': sin, 'Y': cos}

        # Computes distance in long/lat and translates it to pixels in Y direction
        hypotenuse = haversine(upper_left_in["gps"][1], upper_left_in["gps"][0], current_in[1], current_in[0])
        bearing = get_bearing(upper_left_in["gps"][1], upper_left_in["gps"][0], current_in[1], current_in[0])
        currentDistance = direction_func[direction_in](bearing * pi / 180) * hypotenuse

        totalHypotenuse = haversine(upper_left_in["gps"][1], upper_left_in["gps"][0], lower_right_in["gps"][1], lower_right_in["gps"][0])
        bearing = get_bearing(upper_left_in["gps"][1], upper_left_in["gps"][0], lower_right_in["gps"][1], lower_right_in["gps"][0])
        totalDistance = direction_func[direction_in](bearing * pi / 180) * totalHypotenuse
        currentPixel = currentDistance / totalDistance * (lower_right_in["pixel"][direction_index[direction_in]] - upper_left_in["pixel"][direction_index[direction_in]])

        # print(str(currentPixel) + " in direction " + str(direction))
        return currentPixel + upper_left_in["pixel"][direction_index[direction_in]]
    
    def getPixelCoordinates(upper_left_in, lower_right_in, current_in):
        """
            Function to encapsulate getCurrentPixelAxis functionality for both axis in just one call.
            Returns pixel coordinates of a given GPS coordinates based on reference provided.

            :seealso: getPixelCoordinates() function to see param description

            :return: Returns array of two coordinates with pixel coordinates of given point
        """
        coord = [getCurrentPixelAxis(upper_left_in, lower_right_in, current_in, "X"), getCurrentPixelAxis(upper_left_in, lower_right_in, current_in, "Y")]
        return coord
    


    image = plt.imread(filename_in)
    
    # Figure has to be created with image sizes so resolution is not lost
    dpi = 300
    height, width, nbands = image.shape
    figsize = width / float(dpi), height / float(dpi)


    pixel_coord = []
    gps_status = [] # GPS status of current point
    error_ellipse = []
    not_in_image = False # Flag for logging

    m_to_pixel_transform = float(data[0]["pixel"][1] - data[1]["pixel"][1]) / float(haversine(data[0]["gps"][1], 0, data[1]["gps"][1], 0))
    for current in data_in:
        # Are these coords inside the image?
        current_pixel_coord = getPixelCoordinates(data[0], data[1], current["position"])
        if current_pixel_coord[0] > 0 and current_pixel_coord[1] > 0 and \
           current_pixel_coord[0] < width and current_pixel_coord[1] < height:
            pixel_coord.append(current_pixel_coord)
            gps_status.append(current["status"])
            error_ellipse.append([m_to_pixel_transform*sqrt(current["position_covariance"][0])*4,
                                  m_to_pixel_transform*sqrt(current["position_covariance"][1])*4])
        else:
            not_in_image = True

    if not_in_image:
        print ("[WARN] [projectOverMapImage] - Some pixels are not inside image area." )


    
    # Create a figure of the right size with one axes that takes up the full figure
    fig = plt.figure(figsize=figsize)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis('off') # Hide spines, ticks, etc.
    ax.imshow(image, interpolation='nearest') # Display the image.

    # Separates X and Y coordinates into different arrays and assigns a color to it based
    # on the status in which this point was obtained
    x_coord = [i[0] for i in pixel_coord]
    y_coord = [i[1] for i in pixel_coord]
    
    color_dict = {-1: 'r', 0: 'r', 1: 'y', 2: 'g'}
    color_list = [color_dict[current] for current in gps_status]
    ax.scatter(x = x_coord, y = y_coord, marker = '+', c = color_list, s = 1)

    # Computes error ellipse based on 4 times sigma
    width_ellipse = [i[0] for i in error_ellipse]
    height_ellipse = [i[1] for i in error_ellipse]

    ec = EllipseCollection(width_ellipse, height_ellipse, 0, units='xy', offsets=list(zip(x_coord,y_coord)),
                       transOffset=ax.transData, edgecolors = color_list, facecolors = "none")
    ax.add_collection(ec)

    # Plot reference points used for interpolation in blue
    ax.scatter(data[0]["pixel"][0], data[0]["pixel"][1], c='b', s=10, marker = '+')
    ax.scatter(data[1]["pixel"][0], data[1]["pixel"][1], c='b', s=10, marker = '+')

    fig.savefig(filename_in.replace(".jpg","_out.jpg"), dpi=dpi, transparent=True)


    return None


def projectLogsOnMap():
    from data_set_list import data_set_path, map_data_set_list

    for item in map_data_set_list:
        data = []
        for data_path in item["data"]:
            data += parseROSTopicLog(data_set_path + data_path)
        
        for map_path in item["map"]:
            projectOverMapImage(data_set_path +  map_path, data)

if __name__ == "__main__":
    # path = './2022_06_16/'
    # test_config = parseYAMLConfig(path+'test_16_06_2022.yml')

    # for key, item in test_config.items():
    #     if "test" in key:
    #         iter = -1
    #         key_dict = list(item[iter].keys())[0]
    #         point_key = item[iter][key_dict]
    #         file = path + key + "_" + key_dict.lower() + ".log"
            
    #         data = parseROSTopicLog(file)
            
    #         point_google = test_config[point_key]
    #         for current in data:
    #             point = current["position"]
    #             distance = haversine(point[1], point[0], float(point_google[1]), float(point_google[0]))
    #             print("[INFO] [main] - Distance between " + key_dict.lower() +" antenna and point " + point_key + " is: " + str(distance))
    #             break
    # 
    # data = parseROSTopicLog(path + "test1_base.log")
    # img_path = path +  "img/"
    # projectOverMapImage(img_path +  "mapa_zona_1.jpg", data)

    projectLogsOnMap()