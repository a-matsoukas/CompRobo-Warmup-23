# Warmup Project

Course: A Computational Introduction to Robotics, Spring 2023

Professor: Paul Ruvolo

## Directions

In your github repository, create a markdown file called README.md to serve as documentation for your project. Your writeup should answer the following questions. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

- For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach. Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.
- For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors? Consider including a state transition diagram in your writeup.
- How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
- What if any challenges did you face along the way?
- What would you do to improve your project if you had more time?
- What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.

## Project Overview

## Behaviors

- For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach. Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

### Robot Teleop

#### Objective

#### Approach

#### Limitations

#### Tricky Decisions

#### Results

### Driving in a Square

#### Objective

#### Approach

#### Limitations

#### Tricky Decisions

#### Results

### Wall Following

#### Objective

#### Approach

#### Limitations

#### Tricky Decisions

#### Results

#### Objective

The objective of this behavior is to pilot or place the neato near a wall and to have the neato follow the wall at a fixed distance, parallel to the wall. The main tool used in this behavior was the neato's lidar sensor, which returns a list of 360 values, indicating how far objects are from the neato at each degree around it.

#### Approach

<figure
    style=
        "display: block;
        margin-left: auto;
        margin-right: auto;
        width:60%;"
>
    <img 
        src="./Diagrams/wall_follower_diagram.jpg"
        alt="Wall Follower Diagram"
    >
</figure>

My wall following approach was to keep the angle labeled `θ` in the diagram as close to 45° as possible. This is because if the neato is parallel to the wall, the lidar scan at 270° will be perpendicular to the wall; furthermore, the lidar scan at 315° creates a 45° angle with the lidar scan at 270°. These facts imply that when the neato is parallel to the wall, `θ` will be 45°.

The lidar scans at 225° and 315° (distances labeled as `y` and `x` in the diagram, respecively) create a right triangle that allows `θ` to be calculated as follows:

```
θ = atan(y / x)
```

Given that the neato is trying to optimize Scenario: `θ`, there are a few scenarios possible as it follows the wall, which are shown below.

<figure
    style=
        "display: block;
        margin-left: auto;
        margin-right: auto;
        width:60%;"
>
    <img 
        src="./Diagrams/wall_follower_angles.jpg"
        alt="Wall Follower Angles"
    >
</figure>

In this implementation, the neato drives at a constant linear velocity, and its angular velocity is proportional to how far `θ` is from 45°.

### Limitations

In general, the neato can only follow a wall that is on its right side, which means that its starting angle needs to be such that the wall is generally located on its right side. Furthermore, the neato does not respond appropriately when there is a wall directly in front of it or when there is a gap in the wall it is currently following.

Both of these limitations in my implementation come from a trade-off of exploring this behavior as deeply as possible versus having time to sufficiently explore the following behaviors.

### Tricky Decisions

The biggest challenge I encountered with this behavior was transitioning from the simulator to real life; specifically, I did not realize how tricky it would be to get good data from the neatos lidar sensor. Because of this, I needed to build in some redundancy in my angle calculations and base it off of multiple angles as shown below.

<figure
    style=
        "display: block;
        margin-left: auto;
        margin-right: auto;
        width:60%;"
>
    <img 
        src="./Diagrams/wall_follower_extra_measurements.jpg"
        alt="Wall Extra Measurements"
    >
</figure>

Each angle `φ` is calculated using the same `arctan` formula as above, at various lidar angles, denoted as `ψ`. Using the fact that the sum of the angles of the inner triangle must be 180° and solving for `θ`, the following formula is reached:

```
θ = φ - 225 + ψ
```

This method allows for redundancy by calculating `θ` multiple times using different data points. The neato uses the mean of the angle measurements it calculates to make its turning decisions.

#### Results

### Person Following

#### Objective

The objective of this approach is to have the neato follow a person that enters its tracking area (below) and stay a fixed distance away from the person it is following. The main tool used in this implementation is the neato's lidar sensor.

#### Approach

<figure
    style=
        "display: block;
        margin-left: auto;
        margin-right: auto;
        width:60%;"
>
    <img 
        src="./Diagrams/person_follower_diagram.jpg"
        alt="Wall Extra Measurements"
    >
</figure>

The neato's tracking region is defined by an angle `θ` from the neato's 0° and a radius, `r`. From the lidar scan data that the neato recieves, it only utilizes data from the angles within the specified region whose values are less than or equal to `r`.

<figure
    style=
        "display: block;
        margin-left: auto;
        margin-right: auto;
        width:60%;"
>
    <img 
        src="./Diagrams/person_follower_centroid.jpg"
        alt="Wall Extra Measurements"
    >
</figure>

As shown in the diagram above, the neato senses something in its tracking region. The centroid of the points is calculated in polar coordinates (radius, angle to target from the neato's 0°) and used to inform the neato's behavior. If the angle is negative, the neato will turn clockwise, and if the angle is positive, the neato will turn counterclockwise. The speed at which the neato turns is proportional to the angle. Furthermore, the neato's linear velocity is proportional to the distance to the target minus the desired following distance. Thus, if the neato gets too close to the target, or the person starts walking towards the neato, the neato will back up.

#### Limitations

The biggest limitation of this algorithm is what happens if there are multiple objects in its tracking region, or if a wall enters its tracking region. Due to the method of calculating the centriod, if there are two people in its tracking region, for example, it will try to follow a spot in between them. Furthermore, a wall is a dense collection of points on the lidar scan, while a person only shows up as a few points. Because of this, the neato will attempt to 'follow' a wall, if one enters its field of view. Thus, this person-follower works best in ideal conditions, but behaves very poorly otherwise.

#### Tricky Decisions

The toughest part of implementing this behavior was managing the issue described in the previous section. One solution I found to mitigate it a little bit was to limit the angle and radius of the tracking region, so that it doesn't pick up on extranious objects. This worked fairly decently, but also means that the person needs to be very close to the neato to be followed.

#### Results

### Obstacle Avoidance

#### Objective

#### Approach

#### Limitations

#### Tricky Decisions

#### Results

## Finite State Controller

- For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors? Consider including a state transition diagram in your writeup.

## Code Structure

- How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.

## Challenges

- What if any challenges did you face along the way?

## Future Improvements

- What would you do to improve your project if you had more time?

## Key Takeaways

- What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.
