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
    - Configure for compilation/installation: `./waf configure --prefix=path_to_install`
    - Compile with `./waf`
    - Install robdyn on your computer: `sudo ./waf install` (the sudo is required if you are installing the files in the default locations /usr/local/lib and /usr/local/includes).
- [limbo]: A lightweight framework for Bayesian and model-based optimisation of black-box functions
    - As limbo is a framework you only need to get the code: `git clone https://github.com/jbmouret/limbo.git`

### Compiling and Executing the experiment

- Make sure you have all the dependencies installed/downloaded.
- Go to your `limbo` root directory
- Configure for compilation: `./waf configure`
- Compile limbo with: `./waf`
- Create an experiment folder (if there's none) and cd to it: `mkdir exp && cd exp`
- Clone ITE: `git clone git@github.com:resibots/ITE.git`
- Compile the ITE experiment: `./waf --exp ITE`
    - You need to add `--robot true` if you want to run ITE on the real robot


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[paper]: http://www.nature.com/nature/journal/v521/n7553/full/nature14422.html
[robdyn]: https://github.com/resibots/robdyn
[limbo]: https://github.com/resibots/limbo
