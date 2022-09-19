# Robot Arm Control using MATLAB

This project uses MATLAB’s App Designer and Robotics System Toolbox to create a graphical user interface that allows controlling the LSS 4 DoF Robotic Arm. The arm uses the LSS Adapter Board to communicate with and power the Lynxmotion Smart Servos. These servos can be controlled using standard RC PWM or the LSS communication protocol which is what is used in this project.

<div align="center">
  <a href="https://www.youtube.com/watch?v=_YmGmsMpEeQ"><img src="https://github.com/Robotics-Technology/Arm-Control-Matlab-App/images/thumbnail.jpg"  width="70%" alt="MATLAB Arm Control"></a>
</div>

## Table of Contents

- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Open](#open)
- [Run](#run)
- [Author](#author)
- [License](#license)
- [Resources](#resources)

## Requirements

### Hardware

- LSS 4 DoF Robotic Arm
- Windows PC (Tested on 64-bit machine)

### Software

- [MATLAB](https://www.mathworks.com/products/matlab.html) (Tested on R2019a)
- [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html)

## Getting Started

To start using this project, proceed to the standard *clone* procedure:

```bash
cd <some_directory>
git clone https://github.com/Robotics-Technology/Arm-Control-Matlab-App.git
```

## Open

Click on LSS_App.mlapp, once the project is open click on LSS_App.mlapp again an it will open the App Desginer window.

If you want to check the code you can click on "Code View" on the right.

## Run

To run the interface press the play button or F5. 

<p align="center">
  <img src="https://github.com/Robotics-Technology/Arm-Control-Matlab-App/images/MATLAB_LSS_GUI.jpg"/>
</p>

## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## License

Available under the GNU General Public License v3.0

## Contributing

Anyone is very welcome to contribute. Below is a list of identified improvements.

## To do

- Import the arm model from an (URDF)[https://www.mathworks.com/help/robotics/ref/importrobot.html].

## Limitations

- The code doesn't currently support collision check, this is why the pose has to be tested on the interface using the "View" button, if there are no collisions you can then press "Send" to move the physical robot.

## Resources

This project is further explained [here](https://www.robotshop.com/community/robots/show/using-matlab-to-control-a-robotic-arm#).

Read more about the LSS Robotic Arm in the [wiki](https://www.robotshop.com/info/wiki/lynxmotion/view/servo-erector-set-robots-kits/ses-v2-robots/ses-v2-arms/lss-4dof-arm/).

Purchase the LSS arm on [RobotShop](https://www.robotshop.com/en/lynxmotion-smart-servos-articulated-arm.html).

If you want more details about the LSS protocol, go [here](https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/).

Have any questions? Ask them on the Robotshop [Community](https://www.robotshop.com/community/).