# nao-dynamics-imitation

This repository contains the implementation of Dynamic Modelling of NAO robot and imitation of the human actor wearing Xsens suit. Note that the work has been tested in NAO v5 H25 robot with Xsens Analyze on MAC OS Big Sur in M1 Macbook Pro.

# Requirements

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
| Copellia Simulator         | 4.1.0          |
| Choreographe Suite         | 2.1.4          |

# Procedure

- Install CMake, Python, Unix Generator, Copellia Simulator and Choreographe Suite
- Install NAO C++ SDK
- Initialize Worktree
- Configure and Build
- Execute and Run

# Installation

Start by cloning the repository,

```
git clone <git repo id>
```

## 1. NAO SDK Installation

NAO C++ SDK procedure is installed with the same procedure as of,

https://developer.softbankrobotics.com/nao6/naoqi-developer-guide/sdks/c-sdk/c-sdk-installation-guide

- The `config.cmake` of NAO SDK has to be changed to

```
set(CMAKE_CXX_FLAGS "-stdlib=libc++" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_COMPILER /usr/bin/g++ CACHE INTERNAL "" FORCE)
set(CMAKE_C_COMPILER /usr/bin/gcc CACHE INTERNAL "" FORCE)
```

## 2. CMake Installation

CMake and Unix Make Generator are installed using `brew` command

```
brew install cmake make
```

## 3. Python

Python3 should be the default python version as in the time of creating the repository. One can also installed custom python version using `brew` as

```
brew install python
```

> Note: But the python version used here is 3.7 instead of 2.7. To ensure everything is working, the installation is tested in `virtualenv`. It can also be installed systemwide too.

## 4. Copellia Simulator

Install latest version or the preferred version of Copellia simulator from `https://www.coppeliarobotics.com/downloads`.

## 5. Choreographe Suite

In Mac, the installer is not the desirable one. So, the choreographe-suite is installed from the binary version.

If there are any dynamic linker dependency errors, the additional tools listed below can come in handy.

# Worktree Initialization

To initialize the worktree, please run the instruction from belo,

https://developer.softbankrobotics.com/nao6/naoqi-developer-guide/creating-application/creating-new-application-outside-choregraphe-using

# Build

In WorkTree directory, assuming the `qibuild config` and `qitoolchain` has been properly configured, run

```
qibuild configure --release
qibuild make
```

# Execution and Run

## Run in real-time

If you are using your MAC, you need to set the `DYLD_LIBRARY_PATH` to help the executable looking for dynamic shared libraries created during build.

For example,

```
export DYLD_LIBRARY_PATH=/path/to/NAO-worktree/nao_dynamics_imitation/build-nao-config/sdk/lib
```

You need to be in `bin` folder of your `build` directory to run the executable that has been created.

To run the imitation in real robot, run

```
./nao_dynamics_imitation XSENS_IP XSENS_PORT ROBOT_IP ROBOT_PORT ROBOT_SPEED MODE
```

## Run in Simulation

To test the simulation in Vrep, follow the procedure below

1. Start CopelliaSim and open the scene `NAO.ttt`.
2. The default IP and port for CopelliaSim External API is `127.0.0.1` and `19999`.
3. Now to start the Naoqi server, in `choreographe-suite/bin` and run

```
./naoqi-bin -p 9559 &
```

4. The default IP and port for NAOqi server and the simulation robot is `127.0.0.1` and `9559`.
5. Now everything is setup, in `choreographe-suite`, run `./choreographe` and connect to the simulated robot.
6. If you are using your MAC, you need to set the `DYLD_LIBRARY_PATH` to help the executable looking for dynamic shared libraries created during build.

For example,

```
export DYLD_LIBRARY_PATH=/path/to/NAO-worktree/nao_dynamics_imitation/build-nao-config/sdk/lib
```

7. You need to be in `bin` folder of your `build` directory to run the executable that has been created. Finally, run to test the Copellia simulation.

```
./vrep_simulation_test
```

or run the imitation problem in simulation with

```
./nao_dynamics_imitation_sim XSENS_IP XSENS_PORT VREP_IP VREP_PORT ROBOT_IP ROBOT_PORT ROBOT_SPEED MODE
```

# Some Important Tools

These are some of the important tools that come in handy during the testing

- install_name_tool
- otool
