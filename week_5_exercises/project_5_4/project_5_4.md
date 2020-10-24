# Project 5.4
Today we will be working with a particle filter. This project is more about you
understanding what a particle filter is and how it works rather than
implementing your own.

# Background

# Tasks
The only file you need to worry about for this is
the week_5_exercises/project_5_4/world.yaml file shown below.
You only need to worry about the lines with comments on them.

```yaml
limits: &limits
    acceleration:
        linear: 5
        angular: 5
    twist:
        linear: 2
        angular: 2

noisy_imu: &noisy_imu
    accelerometer: 0.1 # change the noise on accelerometer
    gyro: 0.1          # change the noise of the gyro
    magnetometer: 0.02

noisy_oswin: &noisy_oswin
    imu_covariances: *noisy_imu
    state:
        pose:
            position:
                x: -5 # initial starting location of the tutle
                y: 0
        twist:
            linear: 0
            angular: 0
    limits: *limits
    publishers:
        imu: true
        pose: true

worlds:
    imu_test:
        turtles:
            oswin: *noisy_oswin

particle_filter: # all of the parameters for the particle filter
    num_particles: 100 # how many particles you have
    resampling_frequency: 1 # How often you resample particles

    init_x_mean: -5 # the mean of a gaussian to sample the initial x values for particles
    init_x_var: 0.25 # the variance of a gaussian to sample the initial x values for particles
    init_y_mean: 0 # the mean of a gaussian to sample the initial y values for particles
    init_y_var: 0.25 # the variance of a gaussian to sample the initial y values for particles
    init_yaw_mean: 0 # the mean of a gaussian to sample the initial yaw values for particles
    init_yaw_var: 0.1 # the variance of a gaussian to sample the initial yaw values for particles

    imu_sigma_x: 0.05 # variance applied to the motion update in x
    imu_sigma_y: 0.05 # variance applied to the motion update in y
    imu_sigma_yaw: 0.1 # variance applied to the motion update in yaw
    imu_sigma_vx: 0.05 # variance applied to the motion update in velocity in the x direction
    imu_sigma_vy: 0.05 # variance applied to the motion update in velocity in the y direction

    gps_cov: 0.1 # the covariance of the gps measurement

buzzsim_helper:
    gps_mean: 0 # the mean of a gaussian we sample the gps noise from
    gps_variance: 0.1 # variance of a gaussian we sample the noise from
    speed: 0.5 # speed of the turtle
```

We will cover parameters in ROS later on, but the main idea is that I load these
at run time so we don't have to recompile every time we change a parameter.

Alright we should be doing decent at first pass, run your particle filter using

```bash
roslaunch week_5_exercises project_5_4.launch
```

This should open a bunch of windows. Look for the one called rviz, again we will cover
this in more detail later but I have loaded the config for you this time. You should see something
like this.

<img src="https://imgur.com/9VxBV6W.png" width="1170" height="849" />

The blue arrows are the locations of the different particles, the ball on the end of them
is showing you how likely those balls are. If the ball if bright red it is very likely
and if the ball is black it is not very likely. Finally the white arrow is the pose ground
truth pose of the turtle, and the red arrow is the estimate from your particle filter. The pink
ball bouncing around is the estimated location of the turtle from the gps, there is some noise.

## A little more detail: Pose Estimation
We are doing a weighted average of the particles in order to calculate the pose estimate

# Part 1: No Sensors
Now the first thing we want to do is to turn off the sensors using the parameter

```yaml
use_gps: true
```

to

```yaml
use_gps: false
```

Now run the particle filter, what do you see happening? How does the lack of a sensor
effect how we sample particle?


# Part 2: No motion update

change all the parameters below to these values

```yaml
    imu_sigma_x: 0.0 # variance applied to the motion update in x
    imu_sigma_y: 0.0 # variance applied to the motion update in y
    imu_sigma_yaw: 0. # variance applied to the motion update in yaw
    imu_sigma_vx: 0.0 # variance applied to the motion update in velocity in the x direction
    imu_sigma_vy: 0.0 # variance applied to the motion update in velocity in the y direction
```

```yaml
use_gps: true
```

Now we are no longer adding any noise in the motion update. What do you see now?
Why does the particle filter converge to a single value after a couple steps?

# Part 3: Sensor Bias
revert your parameters to the initial ones, let's change the noise parameters.
set the following parameters to zero

```yaml
noisy_imu: &noisy_imu
    accelerometer: 0.0 # change the noise on accelerometer
    gyro: 0.0          # change the noise of the gyro

buzzsim_helper:
    gps_mean: 0 # the mean of a gaussian we sample the gps noise from
    gps_variance: 0.0 # variance of a gaussian we sample the noise from
    speed: 0.5 # speed of the turtle
```

What do you see now? your particle filter should be working really good. We are using
the ground truth state and imu measurement to track. Try decreasing the gps_cov
to a very small value (Not zero). Does this improve your tracking?

Now change the value of the gps_mean to be 0.25

```yaml
gps_mean: 0.25
```

What do you see? why is it happening?

# part 4: Playing around
revert your parameters and jack up the speed to 2.0. Now try to tune your particle filter
to track the pose estimate, what are you changing and why?
