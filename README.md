# Intelligent Trial & Error (Limbo experiment)

#### ITE code for the experiments published in Cully et al. (2015), Nature.

Full reference:
Cully, Antoine, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret. "Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.

- Author-generated [[pdf]](http://www.isir.upmc.fr/files/2015ACLI3468.pdf)
- Nature version (paywall) [[html / pdf]](http://www.nature.com/nature/journal/v521/n7553/full/nature14422.html)

Video (click on it to play):

[![Robots that can adapt like animals](http://img.youtube.com/vi/T-c17RKh3uE/0.jpg)](https://www.youtube.com/watch?v=T-c17RKh3uE "Robots that can adapt like animals")

*Other parts of the experiments published in the paper:*
- [limbo]: A lightweight framework for Bayesian and model-based optimisation of black-box functions
- [Sferes2]: A high-performance, multi-core, lightweight, generic C++98 framework for evolutionary computation.
- [map_elites_hexapod]: MAP-Elites code for the hexapod

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
    - Get the code: `git clone https://github.com/jbmouret/limbo.git`
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
    - You can add `--disable-graphics true` if you want to not build the graphic version (requires OSG)

## How to execute an experiment

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

**Generated MAP Elites archives are located in the** `archives` **directory.**

## Funding

This work has been funded by the ANR Creadapt project (ANR-12-JS03-0009) and the European Research Council (ERC) under the European Union’s Horizon 2020 research and innovation programme (grant agreement number 637972 - ResiBots).


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[robdyn]: https://github.com/resibots/robdyn
[limbo]: https://github.com/resibots/limbo
[ROS]: http://www.ros.org/
[hexa_control]: https://github.com/resibots/hexa_control
[Sferes2]: https://github.com/sferes2/sferes2
[map_elites_hexapod]: https://github.com/resibots/map_elites_hexapod
