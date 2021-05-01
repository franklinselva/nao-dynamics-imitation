# nao-dynamics-imitation

This repository contains the implementation of Dynamic Modelling of NAO robot and imitation of the human actor wearing Xsens suit. Note that the work has been tested in NAO v5 H25 robot with Xsens Analyze on MAC OS Big Sur in M1 Macbook Pro.

## Requirements

| Requirements               | Version        |
| -------------------------- | -------------- |
| Python                     | 2.7 or Python3 |
| Cmake                      | 3.0.2          |
| qibuild                    | 3.14.1         |
| NAOqi C++ SDK              | 2.5.7          |
| Eigen (already imported)   | 3.9.x          |
| qpOASES (already imported) | 3.2.0          |
| Make Generator             | Unix Generator |
| Xsens                      | Analyse        |

## Procedure

- Install CMake, Python and Generator
- Install NAO C++ SDK
- Initialize Worktree
- Configure and Build
- Run

## Installation

Start by cloning the repository,

```
git clone <git repo id>
```

### 1. NAO SDK Installation

NAO C++ SDK procedure is installed with the same procedure as of,

https://developer.softbankrobotics.com/nao6/naoqi-developer-guide/sdks/c-sdk/c-sdk-installation-guide

- The `config.cmake` of NAO SDK has to be changed to

```
set(CMAKE_CXX_FLAGS "-stdlib=libc++" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_COMPILER /usr/bin/g++ CACHE INTERNAL "" FORCE)
set(CMAKE_C_COMPILER /usr/bin/gcc CACHE INTERNAL "" FORCE)
```

### 2. CMake Installation

CMake and Unix Make Generator are installed using `brew` command

```
brew install cmake make
```

### 3. Python

Python3 should be the default python version as in the time of creating the repository. One can also installed custom python version using `brew` as

```
brew install python
```

> Note: But the python version used here is 3.7 instead of 2.7. To ensure everything is working, the installation is tested in `virtualenv`. It can also be installed systemwide too.

## Worktree Initialization

To initialize the worktree, please run the instruction from belo,

https://developer.softbankrobotics.com/nao6/naoqi-developer-guide/creating-application/creating-new-application-outside-choregraphe-using

## Build

In WorkTree directory, assuming the `qibuild config` and `qitoolchain` has been properly configured, run

```
qibuild configure --release
qibuild make
```

## Run

If you are using your MAC, you need to set the `DYLD_LIBRARY_PATH` to help the executable looking for dynamic shared libraries created during build.

For example,

```
export DYLD_LIBRARY_PATH=/path/to/NAO-worktree/nao_dynamics_imitation/build-nao-config/sdk/lib
```

You need to be in `bin` folder of your `build` directory to run the executable that has been created.

```
./nao_dynamics_imitation_xsens --pip <ROBOTIP> --pport <ROBOTPORT>
```
