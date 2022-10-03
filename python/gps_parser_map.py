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
        · match[0] = secs(timestamp)
        · match[1] = status
        · match[2] = latitude
        · match[3] = longitude
        · match[4] = altitude
        · match[5] = position_covariance[0,0] # diagonal of the matrix is parsed
        · match[6] = position_covariance[1,1]
        · match[7] = position_covariance[2,2]
        · match[8] = position_covariance_type
    """
    pattern_reg_str = '    secs: ([0-9]+)\n    nsecs: [0-9]+\n  frame_id: "gps"\nstatus: \n  status: ([0-9-]+)\n  service: [0-9-.]+\nlatitude: ([0-9-.]+)\nlongitude: ([0-9-.]+)\naltitude: ([0-9-.]+)\nposition_covariance: \[([0-9-.]+), [0-9-.]+, [0-9-.]+, [0-9-.]+, ([0-9-.]+), [0-9-.]+, [0-9-.]+, [0-9-.]+, ([0-9-.]+)\]\nposition_covariance_type: ([0-9-])'
    

    data = []
    with open(filename_in) as file:
        file = file.read()
        match_list = re.findall(pattern_reg_str,file)
        for index, match in enumerate(match_list):
            curr_data = {}
            curr_data["secs"] = float(match[0])
            curr_data["status"] = float(match[1])
            curr_data["position"] = [float(match[2]), float(match[3]), float(match[4])]
            curr_data["position_covariance"] = [float(match[5]), float(match[6]), float(match[7])]
            curr_data["position_covariance_type"] = float(match[8])

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

        :return: time_stamp_in_image
    """
    # List to be able to get which timestamped coordinates are inside this current image
    time_stamp_in_image = []

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
            time_stamp_in_image.append(current["secs"])
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
    
    #color_dict = {-1: '#ef9007', 0: '#ef9007', 1: '#f6d317', 2: '#c2e812'}
    color_dict = {-1: 'r', 0: 'r', 1: 'y', 2: 'b'}
    color_list = [color_dict[current] for current in gps_status]
    ax.scatter(x = x_coord, y = y_coord, marker = '+', c = color_list, s = 10)

    # Computes error ellipse based on 4 times sigma
    width_ellipse = [i[0] for i in error_ellipse]
    height_ellipse = [i[1] for i in error_ellipse]

    ec = EllipseCollection(width_ellipse, height_ellipse, 0, units='xy', offsets=list(zip(x_coord,y_coord)),
                       transOffset=ax.transData, edgecolors = color_list, facecolors = "none")
    ax.add_collection(ec)

    # Plot reference points used for interpolation in blue
    ax.scatter(data[0]["pixel"][0], data[0]["pixel"][1], c='g', s=10, marker = '+')
    ax.scatter(data[1]["pixel"][0], data[1]["pixel"][1], c='g', s=10, marker = '+')

    fig.savefig(filename_in.replace(".jpg","_out.jpg"), dpi=dpi, transparent=True)


    return time_stamp_in_image

def plotXYSigmaGraph(filename_in, data_in, timestamp_in_image):
    """
        Plots data in a graph easyer to check.
        :param filename_in: - Path of the image to process.
        :param data_in: - Data array of 2d array with gps coordinates to project over the image
        :param timestamp_in_image: - Timestamps of data that actualli is in the current image

        :return: Returns nothing
    """

    import utm

    # Create a figure of the right size with one axes that takes up the full figure
    fig = plt.figure(figsize=(10,13))

    # set the spacing between subplots
    plt.subplots_adjust(left=0.11,
                    bottom=0.06, 
                    right=0.95, 
                    top=0.95, 
                    wspace=0.3, 
                    hspace=0.2)

    # Separates X and Y coordinates into different arrays and assigns a color to it based
    # on the status in which this point was obtained

    data = [utm.from_latlon(current["position"][0],current["position"][1]) for current in data_in]
    # print(data_in[0])
    # print(data[0])

    east = data[0][0]
    north = data[0][1]

    x_coord = [current[0]-east for current in data]
    y_coord = [current[1]-north for current in data]

    # Relative time in seconds since init of test
    time_coord = [current["secs"] for current in data_in]
    
    #color_dict = {-1: '#ef9007', 0: '#ef9007', 1: '#f6d317', 2: '#c2e812'}
    color_dict = {-1: 'r', 0: 'r', 1: 'y', 2: 'b'}
    color_list = [color_dict[current["status"]] for current in data_in]

    sigma_x = [sqrt(current["position_covariance"][0]) for current in data_in]
    sigma_y = [sqrt(current["position_covariance"][1]) for current in data_in]
    
    ax1 = fig.add_subplot(4,1,1)
    # ax1.scatter(range(len(x_coord)), x_coord, c = color_list, s = 0.5) 
    # ax1.plot(x_coord, c = "#ef9007")
    ax1.set_ylabel("este (m)")
    ax1.grid()

    ax2 = fig.add_subplot(4,1,2, sharex = ax1)
    # ax2.scatter(range(len(sigma_x)), sigma_x, c = color_list, s = 0.5) 
    # ax2.plot(sigma_x, c = "#f6d317")
    ax2.set_ylabel("σ_x (m)")
    ax2.grid()

    ax3 = fig.add_subplot(4,1,3, sharex = ax1)
    # ax3.scatter(range(len(y_coord)), y_coord, c = color_list, s = 0.5) 
    # ax3.plot(y_coord, c = "#1e3f54")
    ax3.set_ylabel("norte (m)")
    ax3.grid()

    ax4 = fig.add_subplot(4,1,4, sharex = ax1)
    # ax4.scatter(range(len(sigma_y)), sigma_y, c = color_list, s = 0.5) 
    # ax4.plot(sigma_y, c = "#45bcee")
    ax4.set_ylabel("σ_y (m)")
    ax4.set_xlabel("Tiempo desde inicio (s)")
    ax4.grid()

    fig.suptitle('Desplazamiento desde 30S ' + str(round(east,3)) + ' ' + str(round(north,3)), fontsize=16)
    for current in range(len(data)-1):
        # If current data is not in the image just skip it
        if time_coord[current] in timestamp_in_image and time_coord[current+1] in timestamp_in_image:
            # More than two seconds without data, do not keep continuous line
            if time_coord[current+1] - time_coord[current] > 2:
                continue

            # set relative timestamp
            time_now = time_coord[current] - time_coord[0]
            time_next = time_coord[current+1] - time_coord[0]
            ax1.plot([time_now, time_next], [x_coord[current],x_coord[current+1]], c = color_list[current+1]) 
            ax2.plot([time_now, time_next], [sigma_x[current],sigma_x[current+1]], c = color_list[current+1]) 
            ax3.plot([time_now, time_next], [y_coord[current],y_coord[current+1]], c = color_list[current+1]) 
            ax4.plot([time_now, time_next], [sigma_y[current],sigma_y[current+1]], c = color_list[current+1]) 

    plt.setp(ax1.get_xticklabels(), visible=False)
    plt.setp(ax2.get_xticklabels(), visible=False)
    plt.setp(ax3.get_xticklabels(), visible=False)

    fig.savefig(filename_in.replace(".jpg","_out_graph.jpg"), transparent=True)

def projectLogsOnMap():
    from data_set_list import data_set_path, map_data_set_list

    for item in map_data_set_list:
        data = []
        for data_path in item["data"]:
            data += parseROSTopicLog(data_set_path + data_path)
        
        for map_path in item["map"]:
            timestamp_points_in_image = projectOverMapImage(data_set_path +  map_path, data)
            plotXYSigmaGraph(data_set_path +  map_path, data, timestamp_points_in_image)

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