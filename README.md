# Daxie – Teleoperate your robot
<!-- ![img](./assets/img.png) -->
<p align="center">
<img src="./assets/img.png" width="500" >
</p>

### Installation
Install `daxie`
```
git clone github.com/suessmann/daxie.git
cd daxie
pip install -e .
```
then, the nasty part. The `lerobot` package has changed the calibration protocol, so for now you need to clone old fork with `daxie` integrated.

```
git clone git@github.com:suessmann/lerobot.git lerobot-daxie
cd lerobot-daxie
pip install -e .
```

then, modify the configs with your bus address and run the script
```
python lerobot/scripts/control_robot.py \
  --robot.type=so101 \
  --robot.cameras='{}' \
  --control.type=teleoperate \
  --control.mobile=true
```

### How to use
1. Download .apk from releases page
2. In settings, set the IP of your host (the laptop that `daxie` is installed on)
3. Run the application, scan the surroundings, place a pin on a surface
4. When ready, press volume up button once, the teleop starts then. 
5. To open gripper, press volume down button.

### Safety Notice
Use the phone teleoperation with caution, since the AR pose estimation depends on lightning conditions and the overall quality of the image. Best and sustainable results require well-lit setups.

Do not come close to the robot in operation, maintain a safe distance. 

### Copyright Notice
Robot files are taken from [Maniskill](https://github.com/haosulab/ManiSkill) by [haosulab](https://github.com/haosulab) and [Stone Tao](https://github.com/StoneT2000) in particular, licensed under CC BY-NC 4.0. Available at: [[link](https://github.com/haosulab/ManiSkill/blob/main/LICENSE)].

The inverse kinematics module relies on [pyroki](https://github.com/chungmin99/pyroki) library by Chung Min Kim et al., licensed under MIT license. Available at: [[link](https://github.com/chungmin99/pyroki/blob/main/LICENSE)].

Augmented Reality software (android) relies on [ARCore SDK](https://github.com/google-ar/arcore-android-sdk) by Google, license is mixed and available [[by this link](https://github.com/google-ar/arcore-android-sdk/blob/main/LICENSE)].

This repo is under GPL-3.0 license. The library is distributed "as is", the authors are not responsible for any damage caused while using it.