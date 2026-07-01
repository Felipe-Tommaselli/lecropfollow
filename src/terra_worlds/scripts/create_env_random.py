# Created by: Hunter Young
# Date: 11/19/17
#
# Script Description:
#     Script is designed to take various commandline arguments making a very simple
#     and user-friendly method to generate an array of (X,Y) coordinates to be used
#   as the centers of corn stalks for loading into ROS Gazebo. (Optional: additionally
#   include a randomized yaw angle or pitch angle to simulate lodging and model irregularity).
#
# Current Recommended Usage: (in terminal)
#     python generate_points.py -o name_of_output_data_file
#
# References:
#     XML file setup - https://stackoverflow.com/questions/3844360/best-way-to-generate-xml

import random
import numpy as np
import os
import shutil
import math
import argparse
import json
import matplotlib.pyplot as plt

from xml.dom import minidom


from trajectory_utils import *
from trajectory_utils2 import*


class EnvCreator:
    def __init__(self):
        self.stalk_models = []
        self.stalk_model_heights= []
        self.stalk_model_biodensities = []

        # Constant parameters
        self.num_variants = 0
        self.num_rows = 7
        self.vert_row_len = 5.0  # (m)
        self.hori_row_len = 5.0
        self.radius = 0.05
        self.stalk_dist = 0.3048
        self.row_dist = 0.76
        self.min_row_len = 4.5  
        self.max_row_len = 5.0  
        self.origin = [0, 0]
        self.gps_origin = [40.123456789,-81.123456789]
        self.stalk_min_emerg_prob = 0.75
        self.stalk_max_emerg_prob = 1.0
        self.stalk_min_height = 0.05
        self.stalk_max_height = 2.0
        self.stalk_height_prob = 1.0
        self.forward_plots = 1
        self.sideward_plots = 1

        self.robot_init_margin = 0.25  # (m)
        self.robot_min_init_x = 3.8
        self.robot_max_init_x = 4.0
        self.robot_max_init_yaw = 0.174533 * 0.5  # 5 degrees in radians
        self.robot_min_init_yaw = -0.174533 * 0.5
        
        self.num_wp = 6  # Number of waypoints per route

        # Unique parameters for each environment
        self.row_lens = np.zeros(7)
        self.gap_dist = 0.0
        self.variant_idx = 0

        self.routes_json = {}
        self.env_config = {}
    
    def create_stalk_variants(self):
        for corn_var in range(2, 21):
            self.stalk_models.append(f"corn_variant_{corn_var}")
        for sorg_var in range(3, 9):
            self.stalk_models.append(f"sorghum_variant_{sorg_var}")
        for i in range(len(self.stalk_models)):
            self.stalk_model_heights.append(2.25)
            self.stalk_model_biodensities.append(1)
        self.num_variants = len(self.stalk_models)

    @staticmethod
    def generate_random_coordinate(radius):
        # Return container
        output = []

        # Range of the choose-able angles
        range_angle = (0, 360)

        # Generate a random radius and angle sample
        r = random.random()*radius
        theta = random.randrange(*range_angle)

        # Calculate X-Y coordinates from the randomized radius and angle
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))

        # Store Variable for output
        output.append(x)
        output.append(y)

        return output


    def generate_random_coordinates(self, radius, num_samples):
        # Storage containers
        xs = []
        ys = []

        # Counter Index
        i = 0

        while i < num_samples:

            # Generate a random X-Y coordinate
            center = self.generate_random_coordinate(radius)

            # Store the X-Y coordinates
            xs.append(center[0])
            ys.append(center[1])

            # Increase counting index
            i += 1

        samples = np.array([xs, ys])
        return samples

    @staticmethod
    def generate_mean_centers(vert_rows, vert_cols, vert_dRow, vert_dCol, vert_origin,
                            hori_rows, hori_cols, hori_dRow, hori_dCol, hori_origin):
        # First generate stalks for vertical rows
        # Define Constants
        origin_x = vert_origin[0]
        origin_y = vert_origin[1]
        index = 0
        vert_num_elements = vert_rows * vert_cols

        # Storage containers
        vert_centers = np.zeros((4, vert_num_elements))

        # For each row determine each stalks information
        for row in range(0, vert_rows):

            # For each stalk determine respective X-Y coordinates
            for stalk in range(0, vert_cols):
                # Calculate and store stalk data for output
                vert_centers[0, index] = origin_y + vert_dCol * stalk  # flip x and y direction (vertical rows align with x axis)
                vert_centers[1, index] = origin_x + vert_dRow * row
                #print centers[0, index]

                # Keep track of the current stalk's crop plot indices (i.e what row and plant)
                vert_centers[2, index] = row
                vert_centers[3, index] = stalk

                # Update counter index
                index += 1
        
        # 10% of the time the horizontal row does not exist
        if random.uniform(0.0, 1.0) < 0.9:
            # Generate stalks for horizontal rows
            origin_x = hori_origin[0]
            origin_y = hori_origin[1]
            index = 0
            hori_num_elements = hori_rows * hori_cols

            # Storage containers
            hori_centers = np.zeros((4, hori_num_elements))

            # For each row determine each stalks information
            for row in range(0, hori_rows):

                # For each stalk determine respective X-Y coordinates
                for stalk in range(0, hori_cols):
                    # Calculate and store stalk data for output
                    hori_centers[0, index] = origin_y + hori_dCol * stalk  # flip x and y direction (horizontal rows align with y axis)
                    hori_centers[1, index] = origin_x + hori_dRow * row

                    # Keep track of the current stalk's crop plot indices (i.e what row and plant)
                    hori_centers[2, index] = row
                    hori_centers[3, index] = stalk

                    # Update counter index
                    index += 1

            centers = np.hstack((vert_centers, hori_centers))
        else:
            centers = vert_centers

        # Output centers
        return centers

    def randomize_mean_centers(self, centers, radius):
        # Define Constants
        numStalks = centers.shape[1]

        # For each stalk
        for stalk in range(0,numStalks):

            # Generate a randomized offset
            offset = self.generate_random_coordinate(radius)

            # Apply offset to current stalk's mean center coordinates
            centers[0,stalk] = centers[0,stalk] + offset[0]
            centers[1,stalk] = centers[1,stalk] + offset[1]

        return centers

    @staticmethod
    def randomize_poses(centers):
        # Define Constants
        numStalks = centers.shape[1]
        range_angle = (0, 360)
        poses = np.zeros((3,numStalks))

        # For each stalk
        for stalk in range(0,numStalks):

            # Generate a randomized angle
            theta = math.radians(random.randrange(*range_angle))

            # Apply offset to current stalk's mean center coordinates
            poses[0,stalk] = 0
            poses[1,stalk] = 0
            poses[2,stalk] = theta

        return poses

    def randomize_row_len(self):
        ''' Randomize the length of each row. Currently, min is 4.7 and max is 5.0
        '''
        for i in range(self.num_rows):
            self.row_lens[i] = random.uniform(self.min_row_len, self.max_row_len)

    # ========================================================
    #                 XML Output Functions
    # ========================================================

    @staticmethod
    def load_plot_info_xml(centers, avg_heights, avg_densities, row_counts, row_widths, row_lengths):

        # Create the XML document object to add elements to
        doc = minidom.Document()

        # Create a root to add child elements to
        root = doc.createElement('robot')
        root.setAttribute('xmlns:xacro','http://ros.org/wiki/xacro')
        doc.appendChild(root)

        # Load each stalk's info into XML elements
        for plot in range(0,centers.shape[1]):

            # Extract array values for easy casting and usage
            plot_index = str(centers[3,plot])
            row_index = str(centers[2,plot])
            x = centers[0,plot]
            y = centers[1,plot]
            plot_id = plot
            # Create XML element
            elem = doc.createElement('xacro:plot')
            # Define element's attributes
            elem.setAttribute('plot_id', str(plot_id))
            elem.setAttribute('plot_range_index', str(plot_index))
            elem.setAttribute('plot_crossrange_index', str(row_index))
            elem.setAttribute('x', str(x))
            elem.setAttribute('y', str(y))
            elem.setAttribute('z', str(avg_heights[plot]/2))
            elem.setAttribute('rotation', str(0))
            elem.setAttribute('row_count', str(row_counts[plot]))
            elem.setAttribute('row_width', str(row_widths[plot]))
            elem.setAttribute('row_length', str(row_lengths[plot]))
            elem.setAttribute('average_height', str(avg_heights[plot]))
            elem.setAttribute('average_density', str(avg_densities[plot]))

            # Add element to root
            root.appendChild(elem)

        # Convert XML document to string for viewing and file writing
        xml_str = doc.toprettyxml(indent="  ")

        return xml_str

    def load_stalk_info_xml(self, centers, poses, plotID, stalkEmergenceP, meanStalkHeight, minModel, maxModel, headerXML, footerXML):

        # Create the XML document object to add elements to
        doc = minidom.Document()

        # Create a root to add child elements to
        root = doc.createElement('robot')
        root.setAttribute('xmlns:xacro','http://ros.org/wiki/xacro')
        doc.appendChild(root)

        # Load each stalk's info into XML elements
        count = range(0,centers.shape[1])
        countn = len(count)
        avg_h = 0
        avg_d = 0
        for stalk in count:
            if(random.random() > stalkEmergenceP ):
                continue

            # Extract array values for easy casting and usage
            stalk_index = str(centers[3,stalk])
            row_index = str(centers[2,stalk])
            x = str(centers[0,stalk])
            y = str(centers[1,stalk])
            r = str(poses[0,stalk])
            p = str(poses[1,stalk])
            yaw = str(poses[2,stalk])
            h = meanStalkHeight*.8 + random.random()*(meanStalkHeight*1.2 - meanStalkHeight*.8)

            min = 0
            max = 2
            model = int(round(min + random.random() * (max - min)))

            scale = h / self.stalk_model_heights[model]
            d = self.stalk_model_biodensities[model] * scale

            avg_d = avg_d + d
            avg_h = avg_h + h

            # Create XML element
            elem = doc.createElement('xacro:corn_stalk')
            # Define element's attributes
            elem.setAttribute('stalk_index', str(stalk_index))
            elem.setAttribute('row_index', str(row_index))
            elem.setAttribute('x', str(x))
            elem.setAttribute('y', str(y))
            elem.setAttribute('z', '${elevation}')
            elem.setAttribute('r', str(r))
            elem.setAttribute('p', str(p))
            elem.setAttribute('yaw', str(yaw))
            elem.setAttribute('scale', '${scale}')
            elem.setAttribute('mesh', '${mesh}')
            elem.setAttribute('plot', str(plotID))
            elem.setAttribute('model',str(self.stalk_models[model]))
            elem.setAttribute('scale', str(scale))
            elem.setAttribute('height', str(h))
            elem.setAttribute('density', str(d))
            # Add element to root
            root.appendChild(elem)

        # Convert XML document to string for viewing and file writing
        xml_str = doc.toprettyxml(indent="  ")

        if(headerXML == False):
            #remove first 2 lines of xml
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]

        if(footerXML == False):
            #remove last line of xml
            xml_str = xml_str[0:xml_str.rfind('\n')]
            xml_str = xml_str[0:xml_str.rfind('\n')]

        avg_d = avg_d / countn
        avg_h = avg_h / countn

        return [xml_str, avg_d, avg_h]

    def load_stalk_info_sdf(self, type_t, centers, poses, plotID, stalkEmergenceP, meanStalkHeight, minModel, maxModel, headerXML, footerXML):
        # Create the XML document object to add elements to
        doc = minidom.Document()

        # Create a root to add child elements to
        root = doc.createElement('sdf')
        root.setAttribute('version','1.5')
        doc.appendChild(root)
        model = doc.createElement('model')
        model.setAttribute('name',str(type_t) + '_plot')
        root.appendChild(model)
        static = doc.createElement('static')
        static.appendChild(doc.createTextNode('true'))
        model.appendChild(static)
        model.appendChild(doc.createTextNode(' '))

        # Load each stalk's info into XML elements
        count = range(0,centers.shape[1])
        countn = len(count)
        avg_h = 0
        avg_d = 0
        h = meanStalkHeight*.8 + random.random()*(meanStalkHeight*1.2 - meanStalkHeight*.8)
        self.variant_idx = int(round(0 + random.random() * (self.num_variants - 1)))
        scale = h / self.stalk_model_heights[self.variant_idx]
        d = self.stalk_model_biodensities[self.variant_idx] * scale

        # Randomize the length of each row
        # If stalks are placed beyond the row length in each row, do not write in sdf (only for vertical rows)
        self.randomize_row_len()  

        for i, stalk in enumerate(count):
            # Skip the stalk for certain gap probability
            if(random.random() > stalkEmergenceP ):
                continue
            
            # Skip the stalk in vertical rows if its x position exceeds corresponding row length
            # In this design every row has 17 stalks originally
            row_idx = int(stalk / 17)
            if stalk < 119 and centers[0, stalk] > self.row_lens[row_idx]:
                continue

            # Extract array values for easy casting and usage
            stalk_index = str(centers[3,stalk])
            row_index = str(centers[2,stalk])
            x = str(centers[0,stalk])
            y = str(centers[1,stalk])
            r = str(poses[0,stalk])
            p = str(poses[1,stalk])
            yaw = str(poses[2,stalk])

            avg_d = avg_d + d
            avg_h = avg_h + h

            m_pose = x + ' ' + y + ' ' + str(0) + ' ' + r + ' ' + p + ' ' + yaw

            # Create XML element
            include = doc.createElement('include')
            uri = doc.createElement('uri')
            uri.appendChild(doc.createTextNode('model://' + str(self.stalk_models[self.variant_idx])))
            name = doc.createElement('name')
            name.appendChild(doc.createTextNode('plant_'+str(plotID)+'_'+str(i)))
            pose = doc.createElement('pose')
            pose.appendChild(doc.createTextNode(str(m_pose)))
            include.appendChild(uri)
            include.appendChild(name)
            include.appendChild(pose)

            # Add element to root
            model.appendChild(include)
            model.appendChild(doc.createTextNode(' '))

        # Convert XML document to string for viewing and file writing
        xml_str = doc.toprettyxml(indent="  ")

        if(headerXML == False):
            #remove first 2 lines of xml
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]

        if(footerXML == False):
            #remove last line of xml
            xml_str = xml_str[0:xml_str.rfind('\n')]
            xml_str = xml_str[0:xml_str.rfind('\n')]
            xml_str = xml_str[0:xml_str.rfind('\n')]

        avg_d = avg_d / countn
        avg_h = avg_h / countn

        return [xml_str, avg_d, avg_h]


    # ========================================================
    #            Main function to create environment
    # ========================================================

    def create_env(self, env_idx):
        # Define output file names
        sdf_name = "model"  # The name of .sdf output file
        field_name = f"env_{env_idx}"  # The name of .field output file

        num_plots = self.forward_plots * self.sideward_plots  # Always only one plot
        plot_width = self.row_dist * (self.num_rows - 1)
        self.gap_dist = 0.76  # Gap between horizontal and vertical plots (m)

        final_xml_str = ""
        shapefile_str = field_name + "\n\n" + "PLOTS\n" + str(1) + "\n\n"
        row_info_str = ""
        all_centers_x = []
        all_centers_y = []
        avg_heights = []
        avg_densities = []
        row_counts = []
        row_widths = []
        row_lengths = []
        index = 0
        plot_center = np.array([plot_width / 2, self.vert_row_len / 2])
        plot_id = index
        stalk_emerg_prob = self.stalk_min_emerg_prob + random.random() * (self.stalk_max_emerg_prob - self.stalk_min_emerg_prob)
        mean_stalk_height = self.stalk_min_height + self.stalk_height_prob * random.random() * (self.stalk_max_height - self.stalk_min_height)
        mean_centers = self.generate_mean_centers(self.num_rows, int(self.vert_row_len / self.stalk_dist + 1), self.row_dist, self.stalk_dist, [plot_center[0] - plot_width/2, plot_center[1] - self.vert_row_len / 2],
                                                  int(self.hori_row_len / self.stalk_dist + 1), 3, self.stalk_dist, self.row_dist, [plot_center[0] - plot_width / 2, plot_center[1] - self.vert_row_len / 2 + self.vert_row_len + self.gap_dist])
        centers = self.randomize_mean_centers(mean_centers, self.radius)
        poses = self.randomize_poses(centers)

        # Load the generated stalk info into xml format to be used by ROS
        mean_stalk_height = self.stalk_min_height + random.random()*(self.stalk_max_height - self.stalk_min_height)
        [xml_str, avg_d, avg_h] = self.load_stalk_info_sdf(field_name, centers,poses, plot_id, stalk_emerg_prob, mean_stalk_height, 0, 5, index == 0, index == (num_plots - 1))
        final_xml_str = final_xml_str + xml_str

        shapefile_str = shapefile_str + str(index) + "\n" + "PLOT" + str(index) + "_NAME" + "\nCORN" + "\nThis plot has candy corn" + "\nROWS\n" + str(self.num_rows) + "\n"

        for r in range(0, self.num_rows):
            x0 = plot_center[0] - plot_width / 2  + self.row_dist * (r - 1)
            x1 = x0
            y0 = plot_center[1] - self.vert_row_len / 2
            y1 = y0 + self.vert_row_len
            (lat0, lon0) = dxy2dlatlon(self.gps_origin[0], self.gps_origin[1], x0, y0)
            (lat1, lon1) = dxy2dlatlon(self.gps_origin[0], self.gps_origin[1], x1, y1)
            shapefile_str = shapefile_str + str(self.gps_origin[0] + lat0) + " " + str(self.gps_origin[1] + lon0) + " " + str(self.gps_origin[0] + lat1) + " " + str(self.gps_origin[1] + lon1) + "\n"

        shapefile_str = shapefile_str + "\n"

        index = index + 1

        if(index == 1):
            all_centers_x = centers[0,:].tolist()
            all_centers_y = centers[1,:].tolist()
        else:
            all_centers_x.extend(centers[0,:].tolist())
            all_centers_y.extend(centers[1,:].tolist())

        avg_heights.append(avg_h)
        avg_densities.append(avg_d)
        row_counts.append(self.num_rows)
        row_lengths.append(self.vert_row_len)
        row_widths.append(self.row_dist)

        # Save .sdf file
        if not os.path.exists(f"../models/env_{env_idx}"):
            os.mkdir(f"../models/env_{env_idx}")
        else:
            shutil.rmtree(f"../models/env_{env_idx}")
            os.mkdir(f"../models/env_{env_idx}")
        with open(f"../models/env_{env_idx}/" + sdf_name + ".sdf", "w") as f:
            f.write(final_xml_str)
        shutil.copyfile("../models/model.config", f"../models/env_{env_idx}/model.config")
        
        # Save .field file
        with open("../sdf/" + field_name + ".field", "w") as f:
            f.write(shapefile_str)
    

    # ========================================================
    #                  Generate Routes
    # ========================================================

    @staticmethod
    def find_parabola_coefficients(p1, p2, p3):
        """ x^2 = ay^2 + by + c
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        # Set up the system of equations
        A = np.array([
            [y1**2, y1, 1],
            [y2**2, y2, 1],
            [y3**2, y3, 1]
        ])
        B = np.array([x1, x2, x3])
        
        # Solve for a, b, and c
        a, b, c = np.linalg.solve(A, B)
        
        return a, b, c
    
    @staticmethod
    def plot_waypoints(env_idx, route_idx, init_x, init_y, noi_waypoints, ref_waypoints):
        ref_x = []
        ref_y = []

        for i in range(len(ref_waypoints)):
            ref_x.append(ref_waypoints[i][0])
            ref_y.append(ref_waypoints[i][1])
        
        plt.figure(0)
        # plt.scatter(noi_x, noi_y, label='Noisy path')
        plt.scatter(ref_x, ref_y, label='Reference path')
        plt.show()

    def create_routes(self, env_idx, num_routes):
        """Create different routes inside one environment
        """
        '''
        routes data structure: 
            [[(init_lane, target_lane), (init_x, init_y, init_yaw), (target_x, target_y), [(wp1), (wp2), ...]], [...], ...]
        
        robot_init_lane: The lane no. for robot initial position
        robot_target_lane: The lane no. for robot target position
        robot_init_x: Robot initial x position (m) in world frame
        robot_init_y: Robot initial y position (m) in world frame
        robot_init_yaw: Robot initial yaw angle (degrees) in world frame
        robot_target_x: Robot target x position (m) in world frame
        robot_target_y: Robot target y position (m) in world frame
        '''
        self.env_config = {}
        self.routes_json = {}
        for i in range(num_routes):
            route_dict = {}  # Dictionary to store route information

            # Detrmine initial lane and target lane
            init_lane = 0
            target_lane = 0
            while True:
                init_lane_ = random.randint(1, self.num_rows - 1)
                target_lane_ = random.randint(1, self.num_rows - 1)
                if abs(target_lane_ - init_lane_) <= 3 and abs(target_lane_ - init_lane_) > 1:
                    init_lane = init_lane_
                    target_lane = target_lane_
                    break
            route_dict["init_lane"] = init_lane
            route_dict["target_lane"] = target_lane

            # Robot initial pose 
            init_x = random.uniform(self.robot_min_init_x, self.robot_max_init_x)
            init_y = self.row_dist * init_lane - random.uniform(self.robot_init_margin, self.row_dist - self.robot_init_margin)
            init_yaw = random.uniform(self.robot_min_init_yaw, self.robot_max_init_yaw)
            route_dict["init_x"] = init_x
            route_dict["init_y"] = init_y
            route_dict["init_yaw"] = init_yaw 

            # Reference initial and target pose
            ref_init_y = self.row_dist * init_lane - self.row_dist / 2
            ref_target_x = init_x
            ref_target_y = self.row_dist * target_lane - self.row_dist / 2
            route_dict["ref_init_x"] = init_x
            route_dict["ref_init_y"] = ref_init_y
            route_dict["ref_target_x"] = ref_target_x
            route_dict["ref_target_y"] = ref_target_y
                                                                                                                                                                           
            # Fill in waypoints
            noi_waypoints = [[self.vert_row_len, self.row_dist * target_lane], [self.robot_min_init_x, self.row_dist * (target_lane - 1)]]
    
            # Define ellipse parameters
            a = (abs(target_lane - init_lane) * self.row_dist) / 2  # Semi-major axis
            b = 0.5  # Semi-minor axis
            h = ref_init_y + (ref_target_y - ref_init_y) / 2  # y-coordinate of the center
            k = self.vert_row_len  # x-coordinate of the center

            # Generate parameter values
            t = np.linspace(0, np.pi, 10)

            # Parametric equations for the ellipse
            y = h + a * np.cos(t)
            x = k + b * np.sin(t)

            if init_lane < target_lane:
                x = np.append(np.append([self.vert_row_len - 0.5, self.vert_row_len - 0.25], x), [self.vert_row_len - 0.25, self.vert_row_len - 0.5])[::-1]
                y = np.append(np.append([ref_target_y, ref_target_y], y), [ref_init_y, ref_init_y])[::-1]
            else:
                x = np.append(np.append([self.vert_row_len - 0.5, self.vert_row_len - 0.25], x), [self.vert_row_len - 0.25, self.vert_row_len - 0.5])
                y = np.append(np.append([ref_init_y, ref_init_y], y), [ref_target_y, ref_target_y])
            ref_waypoints = [[x[i], y[i]] for i in range(x.shape[0])]

            # self.plot_waypoints(env_idx, i, init_x, init_y, noi_waypoints, ref_waypoints)  # Debug purpose
            route_dict["noi_waypoints"] = noi_waypoints
            route_dict["ref_waypoints"] = ref_waypoints

            self.env_config[f"route_{i}"] = route_dict
            self.routes_json[f"route_{i}"] = route_dict
        
        
    # ========================================================
    #                     Save data
    # ========================================================

    def save_config(self, env_idx):
        if not os.path.exists("../configs"):
            os.mkdir("../configs")
        
        if not os.path.exists(f"../configs/env_{env_idx}"):
            os.mkdir(f"../configs/env_{env_idx}")
        else:
            shutil.rmtree(f"../configs/env_{env_idx}")
            os.mkdir(f"../configs/env_{env_idx}")

        json_object = json.dumps(self.routes_json)
        with open(f"../configs/env_{env_idx}/routes_config.json", "w") as outfile:
            outfile.write(json_object)


if __name__ == "__main__":
    # Parse arguments (specify number of environments to generate)
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_env", type=int, default=1, help="Total number of environments to generate")
    parser.add_argument("--num_routes", type=int, default=1, help="Number of routes in one environment")
    args = vars(parser.parse_args())
    num_env = args["num_env"]
    num_routes = args["num_routes"]

    env_creator = EnvCreator()
    env_creator.create_stalk_variants()  # Specify stalk varaints to choose from

    for env_idx in range(num_env):
        env_creator.create_env(env_idx)
        env_creator.create_routes(env_idx, num_routes)
        env_creator.save_config(env_idx)
