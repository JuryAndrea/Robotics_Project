# Robotics Final Project 
#### Group Z: Altrichter Christian, Jury Andrea D'Onofrio and Francesco Huber
GitHub repository: [Click here](https://github.com/Altricch/robomaster_proj.git)

Enjoy our video repository: [Click here](https://usi365-my.sharepoint.com/:f:/g/personal/donofj_usi_ch/EhkavI2BI3RIgTlFAT8el0EBNBjVjIQa2q_OUt_9QJctLQ?e=BJTnAg)

## Remark
We suggest to use a M1 Pro or equivalent processor to run our simulation as we have experienced errors with regards to the sensor reading updates. This could subsequently lead to a faulty execution (e.g. robot does not stop spinning / scanning the perimeter). If not used with an M1, we cannot guarantee correct execution due to the internal logic of our clock.

<br/><br/>
To `launch` our code, we use the provided `robomaster_ros s1.launch` in `cd dev_ws/src/robomaster_ros/robomaster_ros/launch`

## To run the environments

Firstly, you need to clone the repository in `cd dev_ws/src`. Then, you need to open 4 new terminals.

In T1 run the following command to open CoppeliaSim:

` ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/coppeliaSim.sh`

Once CoppeliaSim has started, kindly open one of the respective scences and set the realtime flag.

#### ⚠️ Notes ⚠️
 - Please pay attention to the dynamic dt in the simulation setting of Coppelia, it should be set at 0,0045.
 - Please pay attention to the objects you add, they should be set as collidable and detectable.

In T2 run the following commands to build the project:

1. `cd dev_ws/`

2. `colcon build`

In T3 run the following commands to bridge:

1. `cd dev_ws/src/`

2. `source ~/dev_ws/install/setup.bash`

3. `ros2 launch robomaster_ros s1.launch name:=RM0001 tof_0:=true`

In T4 execute the following commands to run the node:

1. `cd dev_ws/src/`

2. `source ~/dev_ws/install/setup.bash`

3. `ros2 run robomaster_proj robomapper_node2`

You can source T3 and T4 just once. 

#### ⚠️ Be Careful⚠️
You can find your results in the folder called `plot` in `cd dev_ws/src/robomaster_proj/robomaster_proj/plot`

## Scene Selection
We have tested with the following scenes:
- Base_Scene.ttt ⟶ A basic square room with an angular wall section in top left corner
- Base_Scene2.ttt ⟶ Same as previous but changed starting position
- Base_Scene_Plant.ttt ⟶ Same as the first scene but with obstacles
<br/><br/>
- Scene_different_rotation.ttt ⟶ Start with a different starting rotation and walls configuration
- Stress_Test.ttt ⟶ Version of previous scene with unreachable bottom left corner
<br/><br/>
- Stress_Test2.ttt ⟶ Square room with big diagonal section
- Stress_Test3.ttt ⟶ Large Rectangular Room with some obstacles
- Stress_Test4.ttt ⟶ Same as stress test 2 with changed starting position
- Stress_Test5.ttt ⟶ Large Room with irregular walls
- Stress_Test6.ttt ⟶ Same as stress test 5 with changed starting position
<br/><br/>
- Caos.ttt ⟶ Messy room (enjoy!)
