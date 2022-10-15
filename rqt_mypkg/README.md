# rqt_mypkg
Adapted form (https://github.com/how-chen/rqt_mypkg.git). This is a custom control panel in rqt for medical snake robot.

## Executing this program: 
### 1. Clone the repository
```
This repo is uploaded to the group drive (https://drive.google.com/drive/folders/1V-bPo71-b1Did3utUROiP7M6fTFFgzVr).
```

### 2. Specify execution script
```
% roscd rqt_mypkg/scripts
% chmod +x rqt_mypkg
```

note: if roscd does not work, 
```
% cd ~/[workspace_name]
% source ./devel/setup.bash
```

### 3. Run Catkin Make
```
% cd [catkin workspace]
% catkin_make
```
### 4. Start roscore
In a new terminal window
```
% roscore
```

### 5. Run Script
```
% rosrun rqt_mypkg rqt_mypkg
```