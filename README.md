# Intelligent Trial & Error (Limbo experiment)

This package contains code for running the Intelligent Trial & Error Algorithm experiments on our hexapod. See the [paper] for more information:
Cully, Antoine, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret. "Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.


Video:
https://www.youtube.com/watch?v=T-c17RKh3uE

#### This is the code corresponding to the Nature paper experiments.

## Authors
- Original author : Antoine Cully
- Other contributions: Jean-Baptiste Mouret, Konstantinos Chatzilygeroudis

## How to compile

### Dependencies

- [robdyn]: Dynamic simulator
    - Get the code: `git clone https://github.com/jbmouret/robdyn.git`
    - Configure for compilation/installation: `./waf configure`
    - Compile with `./waf`
    - Install robdyn on your computer: `sudo ./waf install`
    - For more advanced options, look at [robdyn]'s repo.
- [limbo]: A lightweight framework for Bayesian and model-based optimisation of black-box functions
    - As limbo is a framework you only need to get the code: `git clone https://github.com/jbmouret/limbo.git`
    - For more advanced options, look at [limbo]'s repo.
- [ROS] \(optional\):
    - If you want to run the experiment on the real robot, you need to have ROS installed. Please check the [official instructions](http://www.ros.org/install/) to download and install ROS.
- [hexa_control] \(optional\):
    - If you want to run the experiment on the real robot, you need to have the `hexa_control` ROS package to control the robot. See [hexa_control]'s repo for instructions.

### Compiling

- Make sure you have all the dependencies installed/downloaded.
- Go to your `limbo` root directory
- Configure for compilation: `./waf configure`
- Compile limbo with: `./waf`
- Create an experiment folder (if there's none) and cd to it: `mkdir exp && cd exp`
- Clone ITE: `git clone git@github.com:resibots/ITE.git`
- Go back to your `limbo` root dir
- Compile the ITE experiment: `./waf --exp ITE`
    - You need to add `--robot true` if you want to run ITE on the real robot

## How to Execute an experiment

**Nature version**

  ~~~
  ./build/exp/ITE/hexa_bomean_variant path_to_archive [matern_kernel_l_value] [leg_indices]
  ~~~
- **Parameters:**
  - [] means that the parameter is optional
  - **the order must be followed**
  - *hexa_bomean_variant* - there are 2 variants: **hexa_bomean_robot** (this exists only if you have compiled with `--robot true`) and **hexa_bomean_graphic**
  - *path_to_archive* - path for the MAP Elites archive to use
      - There are some available archives in the `archives` folder
  - *matern_kernel_l_value* - l value for the matern kernel (optional - 0.4 is the default)
  - *leg_indices* - leg_indices should not be more than 5 in number and in [0-5] range - this parameter is useful only when simulating
  - **Example:** `./build/exp/ITE/hexa_bomean_graphic ./exp/ite/archives/archive.dat 0.5 1`
      - This will run a simulated robot using the "./exp/ite/archives/archive.dat", setting the l value to 0.5 and removing leg 1 from the robot.


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[paper]: http://www.nature.com/nature/journal/v521/n7553/full/nature14422.html
[robdyn]: https://github.com/resibots/robdyn
[limbo]: https://github.com/resibots/limbo
[ROS]: http://www.ros.org/
[hexa_control]: https://github.com/resibots/hexa_control
