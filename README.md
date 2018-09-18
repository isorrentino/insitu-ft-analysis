# Welcome to the insitu-ft-analysis [![Build Status](https://travis-ci.org/robotology-playground/insitu-ft-analysis.svg?branch=master)](https://travis-ci.org/robotology-playground/insitu-ft-analysis)
Data and code for analysis of 6 axis force torque sensors, the majority of the tests are done using the [FTSens](http://wiki.icub.org/wiki/FT_sensor) iCub Facility Force/Torque sensor. 


## Installation 
The scripts in this repo uses the [iDynTree matlab bindings](https://github.com/robotology/idyntree). To estimate wrenches based on the iCub robot model.

## How to ... files
[Here](https://github.com/robotology-playground/insitu-ft-analysis/tree/master/docs/howToFiles) there can be found a series of files aiming to document, in a informal way, how to use the main functionalities of this repository.


### External repositories 

#### quadfit 
The external directory external repositories. It contains a copy of the [quadfit](http://www.mathworks.com/matlabcentral/fileexchange/45356-fitting-quadratic-curves-and-surfaces) toolbox 
for fitting quadratics surfaces. 

#### ftSensCalib
If you have access to the ftSensCalib repository, checkout it in the external directory to make it available to the script into this repo, i.e. from the root of the repo 
you can run this command:
~~~
svn co https://svn.icub.iit.local/repos/mecha/ftSensCalib/trunk/ ./external/ftSensCalib
~~~

### Data storage repositories

#### [icub-insitu-ft-analysis-big-datasets](https://gitlab.com/dynamic-interaction-control/icub-insitu-ft-analysis-big-datasets)
You can download the https://gitlab.com/dynamic-interaction-control/icub-insitu-ft-analysis-big-datasets repository in the data directory to make the data from that repo available: 
~~~
git clone https://gitlab.com/dynamic-interaction-control/icub-insitu-ft-analysis-big-datasets.git ./data/icub-insitu-ft-analysis-big-datasets
~~~
 #### [green-iCub-Insitu-Datasets](https://gitlab.com/dynamic-interaction-control/green-iCub-Insitu-Datasets)
You can download the https://gitlab.com/dynamic-interaction-control/green-iCub-Insitu-Datasets repository in the data directory to make the data from that repo available: 
~~~
git clone https://gitlab.com/dynamic-interaction-control/green-iCub-Insitu-Datasets ./data/green-iCub-Insitu-Datasets
~~~
**Note**: given the size of this repositories ( over 50Gb ), most likely is only desired to pick some of them and not all. For doing this instructions can be found [here](https://stackoverflow.com/questions/4114887/is-it-possible-to-do-a-sparse-checkout-without-checking-out-the-whole-repository).
