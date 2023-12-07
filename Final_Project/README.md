# `ME449` Final Project - Mobile Manipulation
**Author**: Allen Liu
## Project Description
This Project is to use the feedback and feedforward control to control the KUKA youbot to pickup a cube in a predefined world and place it in somewhere else.

## Functions
### 1. `NextState`
#### Inputs
 - `config_vec`: (phi, x, y, theta1-5, alpha1-4), the configuration vector representing current configuration of the youbot
 - `speed_vec`: (u1-4, thetadot1-5), the speed vector that representing the speed of each joint
 - `dt`: The timestep of the simulation
 - `max_speed`: Maximum allowance of the speed

#### Outputs
 - `config_new_vec`: Configuration vector of the next timestep.

#### Example
```
config_vec = [0 0 0 0 0 0 0 0 0 0 0 0]';
speed_vec = [10 10 10 10 0 0 0 0 0]';
dt = 0.01;
max_speed = inf;
config_new_vec = NextState(config_vec, speed_vec, dt, max_speed)
 ```
Example Output:
```
config_new_vec =  
         0
    0.0047
   -0.0000
         0
         0
         0
         0
         0
    0.1000
    0.1000
    0.1000
    0.1000
```
### 2. `KUKAHMatrix`
#### Inputs
 - `phi`: The angle of the chasis configuration
#### Output
 - `H_mat`: The H matrix of the KUKA youbot at configuration phi

#### Example
```
phi = 0;
H_mat = KUKAHMatrix(phi)
```
Example Output:
```
H_mat = 
   
   -8.1053   21.0526  -21.0526
    8.1053   21.0526   21.0526
    8.1053   21.0526  -21.0526
   -8.1053   21.0526   21.0526
```
### 3. `TrajectoryGenerator`
#### Inputs
 - `Tse_init`: Initial configuration of the end-effector in the space frame
 - `Tsc_init`: Initial configuration of the cube in the space frame.
 - `Tsc_final`: Final configuration of the cube in the space frame.
 - `Tce_grap`: The configuratino of the cube in end-effector frame when it is being graped by the gripper.
 - `Tce_standoff`: The configuration of the cube in end-effetor frame when the end-effector is in standoff position.
#### Outputs
 - `config_mat`: The matrix representing the configuration at each timestep.

#### Example
```
Tse_init = [1,0,0,0.1992;0,1,0,0;0,0,1,0.7535;0;0;0;1];
Tsc_init = [1,0,0,1;0,1,0,0;0,0,1,0.025;0;0;0;1];
Tsc_final = [0,1,0,1;-1,0,0,-1;0,0,1,0.025;0;0;0;1];
Tce_grasp = [-0.7071,0,0.7071,0;0,1,0,0;-0.7071,0,-0.7071,0,0,0,0,1];
Tce_standoff = [-0.7071,0,0.7071,0;0,1,0,0;-0.7071,0.2,-0.7071,0,0,0,0,1];
k = 10;
config_mat = TrajectoryGenerator(Tse_init, Tsc_init, ...
Tsc_final, Tce_grasp, Tce_standoff, k);
```
Output:
```
% The config_mat should have all configurations in all time step in format
config_mat (i, :) = [r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz gripper_state]
```
### 4. `FeedbackControl`

#### Input
 - `X`: The current pose
 - `Xd`: The desired pose
 - `Xd_next`: Desired pose at next time step
 - `Kp`: Propotional gain
 - `Ki`: Integral gain
 - `dt`: Time step
#### Output
 - `Ve`: End-effector twist as control variables
 - `Xerr`: Error twist in current time step
 - `Xerr_acc`: Accumulated error twist

#### Example
```
X = [0.170,0,0.985,0.387;0,1,0,0;-0.985,0,0.170,0.570;0,0,0,1];
Xd = [0,0,1,0.5;0,1,0,0;-1,0,0,0.5;0,0,0,1];
Xd_next = [0,0,1,0.6;0,1,0,0;-1,0,0,0.3;0,0,0,1];
Xerr_acc = [0; 0; 0; 0; 0; 0];
Kp = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
Ki = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
dt = 0.01;
[Ve, Xerr, Xerr_acc] = FeedbackControl(X, Xd, Xd_next, Xerr_acc, Kp, Ki, dt);
```

Output:

```
Ve =

         0
         0
         0
   21.4000
         0
    6.4500


Xerr =

         0
    0.1709
         0
    0.0795
         0
    0.1067


Xerr_acc =

         0
    0.6834
         0
    0.3178
         0
    0.4268

```

## How to run the file
Navigate to the `code` directory and open MATLAB under the directory, then open the file names `script_final_project.m` file and click run. All the requested `.csv` file and plot `.jpg` files should be automatically generated and 

## Results
For the best

## Output Logs

```
>> script_final_project
--------------- BEST ---------------
Generating animation csv file.
Writing error plot data.
Done.
--------------- NEW TASK ---------------
Generating animation csv file.
Writing error plot data.
Done.
--------------- OVERSHOOT ---------------
Generating animation csv file.
Writing error plot data.
Done.
>>
```