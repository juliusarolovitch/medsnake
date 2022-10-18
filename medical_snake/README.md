## **Introduction ** ##
This package will allow the user the control the HEBI group medical snake to using few keyboard controls.
As the previous works on this device suggests the steps to make the snake move are : Outer Snake locked, Inner Snake unlocked, Inner Snake moved, Inner snake locked, Outer snake unlocked, Outer snake moved.

## Working  ##
The HEBI group JAVA file is used to interact with the snake. Before connecting to the snake the IP Address on your computer must be altered to 192.168.1.x and the Subnet Mask to 255.255.255.0. After the computer recognizes the connection you can start with the programming of the snake. The snake group has to be initialized first which is used so that Matlab knows about the group of motors it is working on and this group is passed as an argument to different functions. The **snake_init** file must be run first every time the snake is switched on. Then the **get_key2** file is used to get a platform up and running on Matlab which asks a command from the keyboard in the form of a key which is used in turn to call the different functions created in this package itself for executing the commands one by one as required for the motion of the snake according to user's wish. The commands and their effects are also described in the get_key2 file. Clicking any key other than the ones used for controlling the snake's motion would result in escaping out of the platform and returning to the normal mode. The Emergency stop must be pressed if the strings come out of any of the pulleys to prevent further tangling of the string.

```
#!matlab

w = forward
s = backward
d = right turn
a = left turn
o = lifting up
k = going down
y =  tightening outer snake
h =  loosening outer snake
```

##Strings and their Effects##
The following actions were used for different controls of the snake:

* For locking the outer snake - All strings are tightened equally.
* For unlocking the outer snake - All strings are loosened equally.
* For locking the inner snake - The inner snake string is tightened.
* For unlocking the inner snake - The inner snake string is loosened.
* For steering the snake in it's own plane - The top left and top right strings on the outer snake are tightened or loosened opposite to each other with the desired values.
* For steering the snake out of the plane - The top left and top right strings on the outer snake are tightened or loosened with same values and the bottom string on the outer snake is loosened or tightened with opposite values.

## Parameters ##
All the parameters are described in the functions given in the Matlab files. The snake_init assigns a lot of parameters to the snake along with the position and velocity gains. The snake steering along the plane in which the snake lies is done separately to the steering in the direction out of the plane in which the snake lies. The **snake_constants** file contains the list of parameters which are chosen according to the wish of the user and they may be altered as suited to the need of the user. Also the constants used in various files such as **snake_advance** or any other control inputs to the motor can be changed accordingly keeping user's requirement in mind. Some key things associated with the snake are:

* The pulleys on the outer snakes move anticlockwise to tighten it but the pulley for the inner snake moves clockwise for tightening.
* High positive torque values tighten the strings on the outer snake while a high negative torque value tightens the inner snake. However the limits on the torque values are specified in the **snake_constants** functions which should not be crossed.
* Also for using the **snake_RelativePositionCmd** or **snake_RelativePositionTorqueCmd**, negative values for the inner and outer snake slides will move the snake forward while positive values will pull them inside.

The speed of the pulleys can also be altered according to one's need in the **snake_constants** function. The dial on the side of the snake must be kept in mind while planning the motion. Ideally it should stay in the middle of the slot. Also, as the snake moves forward we need to keep loosening the outer snake and tightening the inner snake as the motion progresses as the position of these strings move while the snake is in motion and the inner snake needs to be tightened when the outer snake moves otherwise the snake loses it's orientation. The outer snake also needs to be loose enough so that the head of the snake is followed by the rest of the snake. The current values should never exceed the values that are specified in the **snake_constants** function. The torque and the position values provided to the snake must be accurate otherwise the snake can lose it's orientation or may not perform as expected. The outer/inner snake is also translated in steps to have more control over the torque values associated with the outer and the inner snake in order to have more control over the motion and perform more accurate actuations.

##Different functions##
* snake_init - For initializing the group, and reprogramming the gains if required for the snake. It must be run every time the snake is switched ON
* get_key2 - for launching the keyboard interface
* getkey - for processing the keys pressed in the keyboard interface on the basis of their ASCII values
* snake_advance - for advancing the snake
* snake_lock_unlock - for locking and unlocking of inner and outer snake respectively (snake_lock_Outer,snake_lock_Inner,snake_unlock_Inner,snake_unlock_Outer can also be alternately used).This function uses values 1-4 for tightening and loosening the inner or outer snake by setting the desired position values and torque values of the strings associated
* snake_steer_x - for steering the snake in the plane in which the snake lies
* snake_steer_y - for steering the snake in the plane perpendicular to the plane in which the snake lies
* snake_constants - for defining the constants required for the motion of the snake
* snake_RelativePositionCmd - for setting positions of the various motors of the snake independently
* snake_RelativePositionTorqueCmd -  for setting positions as well as torques of the various motors of the snake independently

##Basic working of all the commands##
The whole framework works on an array of 6 values which is passed around the various functions. These values consist of control values for the various motors in the order
inner_snake_slide,outer_snake_slide,inner_snake_rope,head_wire_Bottom,head_wire_TOP_LEFT,head_wire_TOP_RIGHT]
The user can control the various snake motors independently using this array while calling the respective functions like **snake_RelativePositionCmd**. It is however advised that for a zero command for a given motor NaN value should be used rather than passing zero. The position and torque values of the different motors can be set either by using the **snake_RelativePositionCmd** or **snake_RelativePositionTorqueCmd**. They can also be individually setup without calling the function by following the following steps:

```
#!matlab

cmd = CommandStruct();
cmd.position/velocity/torque =[value1 value2 value3 value4 value5 value6]
group.set(cmd);
```
While these commands will set the required values on the motors a pause of some time might be required if used while running the program to let the motion of the motors to be completed.
The **import us.hebi.sdk.matlab.*;** command must be used for every new file created. It is basically required for the execution of the program.
The feedback **fbk** structure can be used to get note of the position,velocity or the torque of the motors. It however must be updated after every movement to get the correct value using the command **fbk=group.getNextFeedback();**

##Improvements and further work to be made##
Some controls are guided on the basis of intuition and experimental values. The increase of use of calculating the values beforehand would be welcomed. Also since the motion of the snake is basically the movement of a set of points and right now the feedback system is not very proper, the control system can be improved by a lot by incorporating Computer Vision. Also a controller can be interfaced with Matlab to shift the key board interface to the controller. Also one of the major problems faced while the operation of the snake is the strings coming out of the pulley while motion and drooping down of the snake which needs to be controlled. The    can be reduced to some extent by tweaking the torque values used for the loosening the outer snake in **snake_advance** function in the part where the outer snake is moving. There is always a chance for reducing the vibrations produced in the snake while executing various controls as it is something which can not be taken out totally from the system but is needed to be minimized as it is to be used for surgical purposes.