# nao-dynamics-imitation

## Requirements

Requirements  | Version
------------- | -------------
Python  | 2.7 or Python3
Cmake  | 3.0
qibuild | 3.14.1
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

```
qibuild configure
qibuild make
```

## Run

```
./nao_dynamics_imitation_xsens --pip <ROBOTIP> --pport <ROBOTPORT>
```
