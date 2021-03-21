# nao-dynamics-imitation

## Requirements

Requirements  | Version
------------- | -------------
Python  | 2.7 or Python3
Cmake  | 3.0
qibuild | 3.14.1
NAOqi C++ SDK | 2.8.5
Eigen (already imported) | 3.9.x
qpOASES (already imported) | 3.2.0
Make Generator | Unix Generator
Xsens | Analyse


## Procedure


* Install CMake, Python and Generator
* Install NAO C++ SDK   
* Initialize Worktree
* Configure and Build
* Run


## Installation

The installation is tested on M1 Macbook Big Sur.

NAO C++ SDK procedure is installed with the same procedure as of,

https://developer.softbankrobotics.com/nao6/naoqi-developer-guide/sdks/c-sdk/c-sdk-installation-guide

### Note: But the python version used here is 3.7 instead of 2.7. To ensure everything is working, the installation is tested in ```virtualenv```. It can also be installed systemwide too.

To initialize the worktree, please run the instruction from belo,

https://developer.softbankrobotics.com/nao6/naoqi-developer-guide/creating-application/creating-new-application-outside-choregraphe-using

```
git clone <git repo id>
```

## Build

In WorkTree directory,

```
qibuild configure
qibuild make
```

## Run

You need to be in ```bin``` folder of your ```build``` directory to run the executable that has been created.

```
./nao_dynamics_imitation_xsens --pip <ROBOTIP> --pport <ROBOTPORT>
```
