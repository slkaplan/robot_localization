# Robot Localization by Particle Filter
## Luke Nonas-Hunter | Sam Kaplan
### Computational Robotics Fall 2020


This is the second project of Introduction to Computational Robotics at Olin College of Engineering. 

In order to familiarize ourselves with the way robots localize themselves in an environment, we (tried to ) implement(ed) a particle filter using ROS and Gazebo. 

Initially, we were presented with two project directions: given a fairly accurate initial robot position, implement a particle filter to correct drift caused by non-ideal motor encoders (or odometry data) OR try to solve the robot kidnapping problem, where you have no idea where the initial robot position is. 

In real robotics applications, it seems more likely that a given robot would not be given an initial position, so we chose to solve the robot kidnapping problem. 


* [High Level](#high-level)
* [Design Decisions](#design-decisions)
* [Challenges](#challenges)
* [Improvements](#improvements)
* [Lessons Learned](#lessons-learned)

<br>


## High Level

Largely abstracted, our signal process flow looks like this

![control_flow](figs/control_flow.jpg)

<br>

### **Initialize Particles**

Like my mom used to say, we have to start somewhere: so this process places an intital amount of particles within the map bounds. This is done once, at the beginning.
<br><br>

### **Initialize Weights**

In order to compute the liklihood of our robot being at a given particle location, the particles must have weights. Later, these will be assigned based on laser scan data, however when they are initialized we assume all particles to have an equal weight.
<br><br>

### **Normalize**

To treat our particles as a probablility distrubution, the sum of all the weights must equal 1 (i.e. the probability of our robot being at any particle must be 100%). We normalize our distrubution like:

    old_weights = [p.w for p in self.particle_cloud]
        for p in self.particle_cloud:
            p.w = float(p.w)/sum(old_weights)

which assigns each particle weight its unnormalized weight divided by the sum of the unnormalized distribution.
<br><br>

### **Guess Robot Pose**

Based on our normalized weights, we can make an educated guess about where our robot is. The techniques for this vary from simply picking the highest weight particle, to an average of a certain percentage of ideal particles.

We tried a couple different ones, but ultimately ended up picking the highest weighted particle. 
<br><br>

### **Resample**

Resampling is one of the most important parts of a particle filter algorithm. Generally it entails picking some good particles, placing new ones around them, and deleting the others. 

We played around with many different methods. Our current process takes the top N% of particles, keeps them, and resamples the remaing 1 - N% around them using Paul's draw_random_sample() function. 

We decrease our particle count using a sigmoid mapping function, which allows us to specify what number particles we start at, P, the number of particles we stabilize at, and at what iteration of resample we should be at P/2 particles. You can see the method here: 

    def sigmoid_function(value, max_output, min_output, middle, inc=1):
        particle_difference = max_output - min_output
        exponent = inc * (value - (middle/2))
        return int(particle_difference/(1 + np.exp(exponent)) + min_output)
<br><br>

### **Deadreckoning**

We must also account for the fact that our robot is moving. While this seems like a bad thing, in really this helps our algorithm converge. To deadreckon, we must be able to access the new robot pose and the old robot pose. Luckily we can do this pretty easily using ROS. 

We assume that any given robot movement can be described by a rotation Theta1, a movement along a vector facing theta1, and a final rotation Theta2:


![control_flow](figs/deadreckon.jpg)

After our first deadreckon process, weights are no longer assigned randomly, but based on a comparison between the robots laser scan data and what that data would look like at a given particle position. 

Our deadreckoning process is described below: 

    direction = math.atan2(delta_y, delta_x)
    theta_1 = self.transform_helper.angle_diff(direction, self.current_odom_xy_theta[2])
    distance = math.sqrt((delta_x**2) + (delta_y**2))

    for p in self.particle_cloud:
        dx = distance * np.cos(p.theta + theta_1)
        dy = distance * np.sin(p.theta + theta_1)

        p.x += dx + np.random.normal(0,0.001)
        p.y += dy + np.random.normal(0,0.001)
        p.theta += delta_theta + np.deg2rad(np.random.normal(0,0.05))

    self.current_odom_xy_theta = new_odom_xy_theta

One very large pain point was that we operating under the assumption of a correctly implemented deadreckon process for large portion of process. That turned out to be dangerous one. We should have initially tested more rigorously, isolating 
<br><br>

## Design Decisions

1. Our first initial decions was to use addition ROS scafolding (given by pf_scaffold.py). Our learning goals were more focused on the mathematical processes of the particle filter algorithm rather then to learn how to use ROS frame transformations. Thankfully, we were provided with such transformations to begin with. 

2. Second, we had to adjust the given control flow somewhat to adapt for the kidnapping problem. The control flow with a given initial positions looks like: 

PAULS CONTROL FLOW

3. In all honesty, we probably could have used more of that framework, but it just didn't make sense to normalize particle weights that weren't assigned yet. In the interest of readability, we changed it to what we have above. 

4. Another important design decision (which was made fairly late in the process), was to create a separate ROS node which publishes our particle cloud at any given moment. We wanted to create live visual debugger for a long time, but it wasn't absolutely necessary until we needed to see what happening to our probability distrubution during resampling. Making graphs that update themselves is hard, and even harder with such a monolithic block of block like the particle filter. So we abstracted it to a ROS node, which, graph issues aside, worked beautifully. Right now we are only visualizing our resampling step, however this framework can be used to visualize anything from within our particle filter. 

5. We knew that tackling the kidnapping problem would mean we would need to be more conscious of our code efficiency. We took every oppurtunity to gain a percentage point, from using list comprehension instead of regular for loops to numpy matrix math when we compare laser scan values. 



MORE DESIGN DECISIONS @LUKE???

<br>

## Challenges

* Not checking code

* Assumed a greater working knowledge of trigonometry than we actually had...

<br>

## Improvements

The solving of the kidnapping problem depends on code efficiency (more particles = better localization) much more than accurate comparisons of particle data (a la given an initial condition). 

That being said, we could have significantly improved our efficiency by: 

* Manual code profiling to provide insight on which particular sections of code are running slow. Given that information we can work on improving. 
* Using numpy features that allow parallelization of code. This largely depends on the above (i.e. the ability to see exactly what part of our code we need to improve). Otherwise we are optimizing blind. 
* Different methods of "corner cutting" (i.e. comparing every third laser scan point instead of every other one)
* Different methods of resampling. Like above, this is largely arbitrary trial-and-error, and thus a significant time sink. 



## Lessons

* Became much more familiar with the numpy library
* Practiced small step coding practices, with rigorous testing of unknown functions. This lead to a decrease in overall debugging time. 

<img src='./figs/finite_state_controller.gif'>