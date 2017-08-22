# autonomous_mobile_robot
The Autonomous Mobile Robot (AMR) is a capstone project for a robotics specialization program through the University of Pennsylvania.

## Note
I did not write some of the simulation code for Python. That came from the good people at Penn. I take no credit for it. Imlimentation of path planning/perception algorithms and other controls were written by myself for this project.

## Login:
You can ssh via ethernet cable by running: 
`ssh pi@10.1.10.55`
Password: `raspberry`

You can ssh via WiFi by running: 
`ssh pi@192.168.0.55`
Password: `raspberry`

## Running the Robo:
In two termianls run: 
`roslaunch robot_launch robot.launch`

and: 

`roslaunch robot_control robot_control.launch`
