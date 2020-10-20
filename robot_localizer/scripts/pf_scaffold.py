#!/usr/bin/env python3


import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.srv import GetMap
from copy import deepcopy
from std_msgs.msg import Float64MultiArray

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import choice
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField
from helper_functions import TFHelper
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y



    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))


class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self):
        print("RUNNING")
        self.initialized = False        # make sure we don't perform updates before everything is setup
        self.kidnap = True
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 1000      # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model

    

        

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)

        # publish weights for live graph node
        self.weight_pub = rospy.Publisher("/graph_data", Float64MultiArray, queue_size=10)

        # laser_subscriber listens for data from the lidar
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []

        # change use_projected_stable_scan to True to use point clouds instead of laser scans
        self.use_projected_stable_scan = False
        self.last_projected_stable_scan = None
        if self.use_projected_stable_scan:
            # subscriber to the odom point cloud
            rospy.Subscriber("projected_stable_scan", PointCloud, self.projected_scan_received)

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()

        # publish the marker array
        #self.viz = rospy.Publisher('/particle_marker', Marker, queue_size=10)
        #self.marker = Marker()
        self.viz = rospy.Publisher('/particle_marker', MarkerArray, queue_size = 10)
        self.markerArray = MarkerArray()

        self.initialized = True

    # assigns robot pose. used only a visual debugger, the real data comes from the bag file. 
    def update_robot_pose(self, timestamp):
        self.normalize_particles()
        self.particle_cloud.sort(key=lambda particle: particle.w)
        length = len(self.particle_cloud)

        # assigns the robot pose as the average of the top ten particle positions. Seems a little jumpy so subject to change. 
        top_ten = self.particle_cloud[length-10:]
        x = [p.x for p in top_ten]
        y = [p.y for p in top_ten]
        theta = [p.theta for p in top_ten]
        avg_x = sum(x)/len(x)
        avg_y = sum(y)/len(y)
        avg_theta = sum(theta)/len(theta)

        self.robot_pose = self.transform_helper.covert_xy_and_theta_to_pose(avg_x,avg_y,avg_theta)
        self.transform_helper.fix_map_to_odom_transform(self.robot_pose, timestamp)

    def projected_scan_received(self, msg):
        self.last_projected_stable_scan = msg

    # deadreckons particles with respect to robot motion. 
    def update_particles_with_odom(self, msg):
        """ To apply the robot transformations to a particle, it can be broken down into a rotations, a linear movement, and another rotation (which could equal 0)
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            delta_x = delta[0]
            delta_y = delta[1]
            delta_theta = delta[2]
            theta_1 = math.atan2(delta_y, delta_x)
            dist = math.sqrt(delta_x**2 + delta_y**2)
            theta_2 = delta_theta - theta_1
            for p in self.particle_cloud:
                p.theta+= theta_1
                p.x += math.cos(math.radians(p.theta)) * dist
                p.y += math.sin(math.radians(p.theta)) * dist
                p.theta += theta_2

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        # TODO: nothing unless you want to try this alternate likelihood model
        pass

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample.
        """
        # make sure the distribution is normalized
        self.normalize_particles()

        
        p_cloud_length = len(self.particle_cloud)

        self.particle_cloud.sort(key=lambda particle: particle.w, reverse=True)

        num_best = int(p_cloud_length*0.5)
        best_particles = self.particle_cloud[0:num_best]

        norm_weights= [p.w for p in best_particles]
        ideal_particles = choice(best_particles, p_cloud_length-len(best_particles), norm_weights)
        self.particle_cloud = []
        self.particle_cloud = best_particles
        dist = 0.5 # adding a square meter of noise around each ideal particle
        for p in ideal_particles:
            x_pos, y_pos, angle = p.x, p.y, p.theta
            x_particle = np.random.normal(x_pos,0.05)
            y_particle = np.random.normal(y_pos,0.05)
            theta_particle = self.transform_helper.loop_around(np.random.normal(angle,1))
            self.particle_cloud.append(Particle(x_particle, y_particle, theta_particle))
            
            # bound = 0.5
            # rand_x = np.random.uniform(-bound, bound)
            # rand_y = np.random.uniform(-bound, bound)
            # x_particle = rand_x + x_pos
            # y_particle = rand_y + y_pos
            # theta_particle = np.random.randint(0,360)
            
        # self.normalize_particles()
        # norm_weights= [p.w for p in best_particles]
        # self.write_to_txt(norm_weights)
        # rospy.sleep(.1)
        self.normalize_particles()
        norm_weights= [p.w for p in best_particles]
        float_array = Float64MultiArray()
        float_array.data = norm_weights
        self.weight_pub.publish(float_array)

        
        

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        #transform laser data from particle's perspective to map coords
        scan = msg.ranges
        
        for index, p in enumerate(self.particle_cloud):
            total_beam_x = np.ndarray((361,))
            total_beam_y = np.ndarray((361,))
            for theta, dist in enumerate(scan):
                theta_rad = math.radians(theta + p.theta)
                total_beam_x[theta] = p.x + dist * math.cos(theta_rad)
                total_beam_y[theta] = p.y + dist * math.sin(theta_rad)
            
            
            particle_distances = self.occupancy_field.get_closest_obstacle_distance(total_beam_x, total_beam_y)
            cleaned_particle_distances = [np.exp(-dist**2) for dist in particle_distances if(math.isnan(dist)!= True)]
            
            p_d_cubed = np.power(cleaned_particle_distances,3)
            p.w = np.sum(p_d_cubed)
           
            #print("Weight Particle #" + str(index) + ": " + str(p.w))





    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        print("Initial Pose Set")
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ 
        Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                    particle cloud around.  If this input is omitted, the odometry will be used 
        Also check to see if we are attempting the robot kidnapping problem or are given an initial 2D pose
        """

        
        if self.kidnap:
            print("Kidnap Problem")
            x_bound, y_bound = self.occupancy_field.get_obstacle_bounding_box()

            x_particle = np.random.uniform(x_bound[0],x_bound[1],size=self.n_particles)
            y_particle = np.random.uniform(y_bound[0], y_bound[1],size=self.n_particles)
            theta_particle = np.random.randint(0,360,size=self.n_particles)
            
        else:
            print("Starting with Inital Position")
            if xy_theta is None:
                print("No Position Definied")
                xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
            x,y,theta = xy_theta    

            x_particle = np.random.normal(x,0.25,size=self.n_particles)
            y_particle = np.random.normal(y,0.25,size=self.n_particles)
            theta_particle = np.random.randint(0,360,size=self.n_particles)

        self.particle_cloud = [Particle(x_particle[i],\
                                        y_particle[i],\
                                        theta_particle[i]) \
                                for i in range(self.n_particles)]
        
        

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        raw = [p.w for p in self.particle_cloud]
        for p in self.particle_cloud:
            p.w = float(p.w)/sum(raw)
        


    def publish_particles(self, msg):
        """
        Publishes particle poses on the map.
        Uses Paul's default code at the moment, maybe later attempt to publish a visualization/MarkerArray
        """

        particles_conv = []
        
        for num, p in enumerate(self.particle_cloud):
            particles_conv.append(p.as_pose())
        
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles_conv))


            # self.marker_update("map", self.particle_cloud, False)
            # self.viz.publish()
            
    def scan_received(self, msg):
        """ 
        All control flow happens here!
        Special init case then goes into loop
        """
        
        if not(self.initialized):
            # wait for initialization to complete
            return

        # wait a little while to see if the transform becomes available.  This fixes a race
        # condition where the scan would arrive a little bit before the odom to base_link transform
        # was updated.
        self.tf_listener.waitForTransform(self.base_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # calculate pose of laser relative to the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        if not(self.particle_cloud):
            print("Particle Cloud Empty")
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
            self.update_particles_with_laser(msg)
            self.normalize_particles()
            self.update_robot_pose(msg.header.stamp)
            self.resample_particles() 
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            print("UPDATING PARTICLES")
            self.update_particles_with_odom(msg)    # update based on odometry
            if self.last_projected_stable_scan:
                last_projected_scan_timeshift = deepcopy(self.last_projected_stable_scan)
                last_projected_scan_timeshift.header.stamp = msg.header.stamp
                self.scan_in_base_link = self.tf_listener.transformPointCloud("base_link", last_projected_scan_timeshift)

            self.update_particles_with_laser(msg)   # update based on laser scan
            self.update_robot_pose(msg.header.stamp)                # update robot's pose
            self.resample_particles()               # resample particles to focus on areas of high density
            
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg)
        
    def marker_update(self, frame_id, p_cloud, delete):
        num = 0
        if(delete):
            self.markerArray.markers = []
        else:
            for p in p_cloud:
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "my_namespace"
                marker.id = num
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose = p.as_pose()
                marker.pose.position.z = 0.5
                marker.scale.x = 1.0
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0 # Don't forget to set the alpha!
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                num+=1

                self.markerArray.markers.append(marker)

    
    def write_to_txt(self, prob_dist):
        
        file_object = open(r"resample_prob.txt","r+")
        file_object.truncate(0)
        for i in range(len(prob_dist)):
            file_object.write(str(prob_dist[i])+"\n")

        
        file_object.close() 


if __name__ == '__main__':
    n = ParticleFilter()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        n.transform_helper.send_last_map_to_odom_transform()

        r.sleep()
