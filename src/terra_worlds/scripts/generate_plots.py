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
import matplotlib.pyplot as plt
import math
import argparse
from xml.dom import minidom
from trajectory_utils import *
from trajectory_utils2 import*

# ========================================================
#             Coordinate Generation Functions
# ========================================================


#stalk_models=["sorghum_variant_6" , "sorghum_variant_7", "sorghum_variant_8"]
stalk_models = ["tobacco_variant_8","tobacco_variant_8","tobacco_variant_8"]
# stalk_models=["simple_corn_stalk_early" , "simple_corn_stalk_early", "simple_corn_stalk_late", "simple_corn_stalk_late", "simple_corn_stalk_late", "simple_corn_stalk_late"]
# stalk_models=["baby_corn_stalk" , "small_corn_stalk", "adult_corn_stalk_variant_0", "adult_corn_stalk_variant_1", "adult_corn_stalk_variant_2", "adult_corn_stalk_variant_3"]
#stalk_models=["corn_variant_0" , "corn_variant_1", "corn_variant_2", "corn_variant_3", "corn_variant_4", "corn_variant_5"]
#stalk_models=["corn_variant_2", "corn_variant_3", "corn_variant_4", "corn_variant_5"]
#stalk_model_season=["early", "early" ,"late","late","late","late"]
# stalk_model_season=["late","late","late"]
#stalk_model_heights=[.25, .5, 2, 2, 2, 2.25]
stalk_model_heights=[2.25, 2.25, 2.25]
# stalk_model_biodensities = [1, 1, 1, 1, 1, 1]
stalk_model_biodensities = [1, 1, 1]
plot_centers = []

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


def generate_random_coordinates(radius, num_samples):

    # Storage containers
    xs = []
    ys = []

    # Counter Index
    i = 0

    while i < num_samples:

        # Generate a random X-Y coordinate
        center = generate_random_coordinate(radius)

        # Store the X-Y coordinates
        xs.append(center[0])
        ys.append(center[1])

        # Increase counting index
        i += 1

    samples = np.array([xs, ys])
    return samples

def generate_mean_centers(rows, cols, dRow, dCol, origin):

    # Define Constants
    origin_x = origin[0]
    origin_y = origin[1]
    index = 0
    num_elements = rows * cols
    #print num_elements
    #print dRow
    #print dCol

    # Storage containers
    centers = np.zeros((4,num_elements))

    # For each row determine each stalks information
    for row in range(0,rows):

        # For each stalk determine respective X-Y coordinates
        for stalk in range(0,cols):
            # Calculate and store stalk data for output
            centers[0, index] = origin_x + dRow * row
            centers[1, index] = origin_y + dCol * stalk
            #print centers[0, index]

            # Keep track of the current stalk's crop plot indices (i.e what row and plant)
            centers[2,index] = row
            centers[3,index] = stalk

            # Update counter index
            index += 1

    # Output centers
    return centers

def randomize_mean_centers(centers, radius):

    # Define Constants
    numStalks = centers.shape[1]

    # For each stalk
    for stalk in range(0,numStalks):

        # Generate a randomized offset
        offset = generate_random_coordinate(radius)

        # Apply offset to current stalk's mean center coordinates
        centers[0,stalk] = centers[0,stalk] + offset[0]
        centers[1,stalk] = centers[1,stalk] + offset[1]

    return centers

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

# ========================================================
#                 XML Output Functions
# ========================================================

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

def load_stalk_info_xml(centers, poses, plotID, stalkEmergenceP, meanStalkHeight, minModel, maxModel, headerXML, footerXML):

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

        # if(h < 1):
        #     min = 0;
        #     max = 1;
        # else:
        #     min = 2;
        #     max = 5;
        min = 0
        max = 2


        model = int(round(min + random.random()*(max - min)))

        scale = h / stalk_model_heights[model]
        d = stalk_model_biodensities[model] * scale

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
        elem.setAttribute('model',str(stalk_models[model]))
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

def load_stalk_info_sdf(type_t, centers, poses, plotID, stalkEmergenceP, meanStalkHeight, minModel, maxModel, headerXML, footerXML):

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
    # if(h < 1):
    #     min = 0;
    #     max = 1;
    # else:
    #     min = 2;
    #     max = 5;

    min = 0
    max = 2


    mesh = int(round(min + random.random()*(max - min)))

    scale = h / stalk_model_heights[mesh]
    d = stalk_model_biodensities[mesh] * scale


    for i,stalk in enumerate(count):
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
        # h = meanStalkHeight*.8 + random.random()*(meanStalkHeight*1.2 - meanStalkHeight*.8)

        # if(h < 1):
        #     min = 0;
        #     max = 1;
        # else:
        #     min = 2;
        #     max = 5;

        # mesh = int(round(min + random.random()*(max - min)))

        # scale = h / stalk_model_heights[mesh]
        # d = stalk_model_biodensities[mesh] * scale

        avg_d = avg_d + d
        avg_h = avg_h + h

        m_pose = x + ' ' + y + ' ' + str(0) + ' ' + r + ' ' + p + ' ' + yaw;


        # Create XML element
        include = doc.createElement('include')
        uri = doc.createElement('uri')
        uri.appendChild(doc.createTextNode('model://' + str(stalk_models[mesh])))
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
#                         Main
# ========================================================

#python generate_plots.py --radius=

def main():
    # Setup commandline argument(s) structures
    ap = argparse.ArgumentParser(description='Extract images from a video file.')
    ap.add_argument("--radius", "-r",           type=float, default='0.1', help="Radius of the bounding circle with which to pull random samples from")
    ap.add_argument("--rows", "-n",             type=int,   default='4', help="Number of rows in each plot to simulate")
    ap.add_argument("--length", "-m",           type=float,   default='10', help="Length of each plot")
    ap.add_argument("--stalk-max-emergence-p", "-s1",    type=float, default='1', help="Max probability of stalk emergence in plot")
    ap.add_argument("--stalk-min-emergence-p", "-s2",    type=float, default='.75', help="Min probability of stalk emergence in plot")
    ap.add_argument("--stalk-min-height", "-s3",    type=float, default='.05', help="Min height mean of stalk in plot")
    ap.add_argument("--stalk-max-height", "-s4",    type=float, default='2', help="Max height mean of stalk in plot")
    ap.add_argument("--stalk-height-p", "-s5",    type=float, default='1', help="Max height mean of stalk in plot")
    ap.add_argument("--stalk-min-model-p", "-s6",    type=float, default='0', help="Min model mean of stalk in plot")
    ap.add_argument("--stalk-max-model-p", "-s7",    type=float, default='5', help="Max model mean of stalk in plot")
    ap.add_argument("--row-spacing", "-S1",      type=float, default='0.76', help="Spacing between each row")
    ap.add_argument("--stalk-spacing", "-S2",    type=float, default='0.3048', help="Spacing between each stalk within a row")
    ap.add_argument("--forward-plots", "-S3",    type=int, default='2', help="number of plots in forward dimension") # number of plots in forward dimension
    ap.add_argument("--sideward-plots", "-S4",    type=int, default='2', help="number of plots in cross-range dimension") # number of plots in cross-range dimension
    ap.add_argument("--forward-plot-separation", "-S5",    type=float, default='1', help="plot separation in forward dimension") # plot separation in forward dimension
    ap.add_argument("--sideward-plot-separation", "-S6",    type=float, default='0.76', help="plot separation in cross-range dimension") # plot separation in cross-range dimension
    ap.add_argument("--origin", "-O", nargs='+',type=float, default=[0,0], help="xy origin ")
    ap.add_argument("--gps-origin", "-gO", nargs='+',type=float, default=[40.123456789,-81.123456789], help="lat lon origin")
    ap.add_argument("--output", "-o",           type=str,   default="generated_crop_plot_field", help="Name of output file containing randomized stalk centers")#generated_crop_plot_field(.urdf.xacro)
    ap.add_argument("--name", "-name",           type=str,   default="generated_crop_plot_field", help="Name of output field file")#generated_crop_plot_field(.field)
    # Store parsed arguments into array of variables
    args = vars(ap.parse_args())

    # Extract stored arguments array into individual variables for later usage in script
    radius = args["radius"]
    numRows = args["rows"]
    plotLength = args["length"]
    dRow = args["row_spacing"]
    dStalk = args["stalk_spacing"]
    origin = args["origin"]
    gps_origin = args["gps_origin"]
    outFile = args["output"]
    stalkMinEmergenceP = args["stalk_min_emergence_p"]
    stalkMaxEmergenceP =args["stalk_max_emergence_p"]
    stalkMinHeight = args["stalk_min_height"]
    stalkMaxHeight = args["stalk_max_height"]
    stalkHeightP = args["stalk_height_p"]
    stalkMinModelP = args["stalk_min_model_p"]
    stalkMaxModelP = args["stalk_max_model_p"]
    forwardPlots = args["forward_plots"]
    sidewardPlots = args["sideward_plots"]
    forwardPlotSeparation = args["forward_plot_separation"]
    sidewardPlotSeparation = args["sideward_plot_separation"]
    name = args["name"]

    numPlots = forwardPlots * sidewardPlots

    plotWidth = dRow * (numRows-1)

    plot_centers = generate_mean_centers(sidewardPlots, forwardPlots, sidewardPlotSeparation + plotWidth, forwardPlotSeparation + plotLength, [origin[0] + plotWidth/2, origin[1] + plotLength / 2])

    final_xml_str = ""
    shapefile_str = name + "\n\n" + "PLOTS\n" + str(plot_centers.shape[1]) + "\n\n"
    row_info_str = ""
    all_centers_x = []
    all_centers_y = []
    avg_heights = []
    avg_densities = []
    row_counts = []
    row_widths = []
    row_lengths = []
    index = 0
    #print plot_centers[0,:]
    #print plot_centers[1,:]
    plot_centers = np.transpose(plot_centers)
    for pc in plot_centers:
        plotID = index
        stalkEmergenceP = stalkMinEmergenceP + random.random()*(stalkMaxEmergenceP - stalkMinEmergenceP)
        meanStalkHeight = stalkMinHeight + stalkHeightP*random.random()*(stalkMaxHeight - stalkMinHeight)
        # centers = generate_random_coordinates(radius,num_samples)
        mean_centers = generate_mean_centers(numRows,int(plotLength / dStalk + 1),dRow,dStalk,[pc[0] - plotWidth/2, pc[1] - plotLength / 2])
        centers = randomize_mean_centers(mean_centers, radius)
        poses = randomize_poses(centers)

        # Load the generated stalk info into xml format to be used by ROS
        meanStalkHeight = stalkMinHeight + random.random()*(stalkMaxHeight - stalkMinHeight)
        [xml_str, avg_d, avg_h] = load_stalk_info_sdf(name, centers,poses, plotID, stalkEmergenceP, meanStalkHeight, 0, 5, index == 0, index == (numPlots-1))
        final_xml_str = final_xml_str + xml_str

        shapefile_str = shapefile_str + str(index) + "\n" + "PLOT" + str(index) + "_NAME" + "\nCORN" + "\nThis plot has candy corn" + "\nROWS\n" + str(numRows) + "\n"

        for r in range(0, numRows):
            x0 = pc[0] - plotWidth/2  + dRow * (r-1)
            x1 = x0
            y0 = pc[1] - plotLength/2
            y1 = y0 + plotLength
            (lat0, lon0) = dxy2dlatlon(gps_origin[0], gps_origin[1], x0, y0)
            (lat1, lon1) = dxy2dlatlon(gps_origin[0], gps_origin[1], x1, y1)
            shapefile_str = shapefile_str + str(gps_origin[0] + lat0) + " " + str(gps_origin[1] + lon0) + " " + str(gps_origin[0] + lat1) + " " + str(gps_origin[1] + lon1) + "\n"

        shapefile_str = shapefile_str + "\n"

        index = index + 1

        #print centers[0,:]
        #print '\n\n\n\n\n\n'

        if(index == 1):
            all_centers_x = centers[0,:].tolist()
            all_centers_y = centers[1,:].tolist()
        else:
            all_centers_x.extend(centers[0,:].tolist())
            all_centers_y.extend(centers[1,:].tolist())

        avg_heights.append(avg_h)
        avg_densities.append(avg_d)
        row_counts.append(numRows)
        row_lengths.append(plotLength)
        row_widths.append(dRow)

    #final_xml_str = final_xml_str + "\n123456789end123456789\n" + load_plot_info_xml(plot_centers, avg_heights, avg_densities, row_counts, row_widths, row_lengths)
    with open("../sdf/" + outFile + ".sdf", "w") as f:
        f.write(final_xml_str)

    with open("../sdf/" +name + ".field", "w") as f:
        f.write(shapefile_str)
# Load Points into plot for visualization
    #print all_centers_x
    #print all_centers_y
    plt.plot(all_centers_x,all_centers_y,'r.')
        # Define dynamic axis limits for showing whole crop plot
    xLowerLimit = origin[0] - 1
    yLowerLimit = origin[1] - 1
    xUpperLimit = sidewardPlots*(plotWidth + sidewardPlotSeparation ) + 1
    yUpperLimit = forwardPlots*(plotLength + forwardPlotSeparation) + 1
        # Update plot axes
    plt.axis([xLowerLimit, xUpperLimit,yLowerLimit,yUpperLimit])
        # Show plot
    plt.show()

main()
