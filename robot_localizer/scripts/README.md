# Robot Localization by Particle Filter
## Luke Nonas-Hunter | Sam Kaplan
### Computational Robotics Fall 2020


This is the second project of Introduction to Computational Robotics at Olin College of Engineering. 

In order to familiarize ourselves with the way robots localize themselves in an environment, we implemented a particle filter using ROS and Gazebo. 

Initially, we were presented with two project directions: given a fairly accurate intial robot position, implement a particle filter to correct drift caused by non-ideal motor encoders (or odometry data) OR try to solve the robot kidnapping problem, where you have no idea where the initial robot position is. 

In real robotics applications, it seems more likely that a given robot would not be given an initial position, so we chose to solve the robot kidnapping problem. 


* [High Level](#high-level)
* [Design Decisions](#design-decisions)
* [Challenges](#challenges)
* [Improvements](#improvements)
* [Lessons Learned](#lessons-learned)



## High Level

Largely abstracted, our signal process flow looks like this

PICTURE OF CUSTOM INFORMATION FLOW

### **Initialize Particles**

Like my mom used to say, we have to start somewhere: so this process places an intital amount of particles within the map bounds. This is done once, at the beginning.

### **Initialize Weights**

In order to compute the liklihood of our robot being at a given particle location, the particles must have weights. Later, these will be assigned based on laser scan data, however when they are initialized we assume all particles to have an equal weight.

### **Normalize**

To treat our particles as a probablility distrubution, the sum of all the weights must equal 1 (i.e. the probability of our robot being at any particle must be 100%). We normalize our distrubution like:

INSERT NORMALIZE EQUATIONS

### **Guess Robot Pose**

Based on our normalized weights, we can make an educated guess about where our robot is. The techniques for this vary from simply picking the highest weight particle, to an average of a certain percentage of ideal particles.

We did INSERT WHAT WE DID

### **Resample**

Resampling is one of the most important parts of a particle filter algorithm. Generally it entails picking some good particles, placing new ones around them, and deleting the others. 

We chose to INSERT WHAT WE DID AFTER WE DECIDE WHAT TO DO

### **Deadreckoning**

We must also account for the fact that our robot is moving. While this seems like a bad thing, in really this helps our algorithm converge. To deadreckon, we must be able to access the new robot pose and the old robot pose. Luckily we can do this pretty easily using ROS. 

INSERT DIAGRAM THAT MAY BE A DIRECT RIP OFF OF PAULS

AFter our first deadreckon process, weights are no longer assigned randomly, but based on a comparison between the robots laser scan data and what that data would look like at a given partile position. 

GIVE MATH AND/OR DIAGRAM

## Design Decisions

Our first initial decions was to use addition ROS scafolding (given by pf_scaffold.py). Our learning goals were more focused on the mathematical processes of the particle filter algorithm rather then to learn how to use ROS frame transformations. Thankfully, we were provided with such transformations to begin with. 

Second, we had to adjust the given control flow somewhat to adapt for the kidnapping problem. The control flow with a given initial positions looks like: 

PAULS CONTROL FLOW

In all honesty, we probably could have used more of that framework, but it just didn't make sense to normalize particle weights that weren't assigned yet. In the interest of readability, we changed it to what we have above. 

MORE DESIGN DECISIONS @LUKE???

## Challenges

Not checking code
update from odom

## Improvements

The solving of the kidnapping problem depends on code efficiency (more particles = better localization) much more than accurate comparisons of particle data (a la given an initial condition). 

That being said, we could have significantly improved our efficiency by: 

* Manual code profiling to provide insight on which particular sections of code are running slow. Given that information we can work on improving. 
* Using numpy features that allow parallelization of code. This largely depends on the above (i.e. the ability to see exactly what part of our code we need to improve). Otherwise we are optimizing blind. 
* Different methods of "corner cutting" (i.e. comparing every third laser scan point instead of every other one)
* Different methods of resampling. Like above, this is largely arbitrary trial-and-error, and thus a significant time sink. 



## Lessons

What the fuck did we learn Luke?