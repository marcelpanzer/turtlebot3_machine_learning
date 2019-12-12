# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

## Run the code:
roslaunch turtlebot3_dqn simulation_...... .launch

depending on the RL method in the launch files


Adaptation of the respawn algorithm to the respective simulated world:

In the simulation_environment_stage_1 file (src/turtlebot3_dqn) substitute the number (here 2)

from simulation_stage_2_respawnGoal import Respawn

with the desired simulated stage number (1-6, set target number to zero when training starts from scratch)
