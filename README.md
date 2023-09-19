# ADCS Toolbox

<img src="https://upload.wikimedia.org/wikipedia/commons/1/18/ISO_C%2B%2B_Logo.svg" width = "16" alt="C++ 14"> <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/1/13/Cmake.svg/900px-Cmake.svg.png" width = "16" alt="CMake"> <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b0/NewTux.svg/800px-NewTux.svg.png" width ="16" alt="Linux">
<img src="https://upload.wikimedia.org/wikipedia/commons/d/d1/Eigen_Silly_Professor_135x135.png" width ="16" alt="Eigen (C++ library)">
<img src="https://upload.wikimedia.org/wikipedia/commons/c/cd/Boost.png" width ="40" alt="BOOST (C++ library)">

High performance <i>Spacecraft Attitude Dynamics and Control System</i> toolbox with exposed python extention. Scalable toolbox uses Eigen3 for linear algebra and Boost for integration of system ODE . Currently there are three sample systems and more can be added provided that new systems are inherited from `IBaseSystem` base interface class and follow same structure. The <b>`pyadcs`</b> extention expose C++ classes and functions of <b>`ADCS`</b> library to python.

1. Compiled final report in pdf format [
tessi_DEORE_1823670.pdf](https://github.com/siddharthdeore/VSCMGThesisReport/releases/download/Defence/tessi_DEORE_1823670.pdf)
2. System URDF [vscmg_description](https://github.com/siddharthdeore/vscmg_description)

# Systems
- Rigid Body - Simple free floating rigid body.
- VSCMG - Variable Speed Control Moment Gyroscope
- RW4 - Satellite with tripod configration of 4 reaction wheels


# Build

Install Linux dependencies
```
sudo apt-get install -y libboost-all-dev 
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y python3 python3-pip
```
Make and install
```
mkdir build && cd build
cmake ..
make
make install
```

# Uninstall adcs library
```
make uninstall
```

# Examples
Executable targets are located in `bin` directory
```
# Rigid Body 
cd build/bin
./TestRigidBody

# Variable Speed Control Moment Gyroscope
cd build/bin
./TestVSCMG

```
# Python Interface
Build with python interface wrapper library. Following comand executed in build directory shall build python interface library `pyadcs.so`  in `bin` directory on linux or `pyadcs.dll` on windows.
```console
cmake .. -DBUILD_PYTHON_LIB=True
make
```
To install python interface library with pip, in projecto root directory 
```
pip install .
```
To uninstall python interface library 
```
pip uninstall adcs
```

# Run python samples
To use library modules in python install library with `make install`, and navigate to `bin` directory and run sample scripts `rigid_body.py` or `vscmg.py`,  eg.

```
python rigid_body.py
```

## Source Tree
```console
.
├── include
│   └── ADCS                     # ADCS Core utils and interface
│       ├── Core
│       │   ├── Controllers.h
│       │   ├── IBaseSystem.h
:       :         :
│       │
│       └── Systems             # Public headers for Systems
│           ├── RigidBody.h
│           ├── RW4.h
│                 :
└── src
    └── ADCS
        ├── py_controller.cpp    # Python wrapper for controller
        └── Systems              # targets in Systems are classes 
            ├── RigidBody.cpp    # inherited from IBaseSystem
            ├── RW4.cpp
            └── VSCMG.cpp

```
## Requirements
- C++ 14
- [Boost](https://www.boost.org/)
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- Python 2.7+


# Docker
```
# Pull latest image from dockerhub
docker pull siddharthdeore/vscmgcpp:latest

# run container in iteractive mode
docker run --rm -it  siddharthdeore/vscmgcpp:latest


# Build image from source
docker build --pull --rm -f "Dockerfile" -t vscmgcpp:latest "."
# or
# docker compose -f "docker-compose.yml" down


# run python examples e.g. vscmg
python3 vscmg.py
```
⚠️ Note : This toolbox was written for MSc Thesis titled <i> Neural Network based steering and Hardware in Loop Simulation of Variable Speed Control Moment Gyroscope</i>

## Cite

1. Attitude control of a fast-retargeting agile nanosatellite using Neural Network based steering for Variable Speed Control Moment Gyroscopes}

```bibtex
@inproceedings{inproceedings,
author = {Deore, Siddharth and Santoni, Fabio and Piergentili, Fabrizio and Marzioli, Paolo},
year = {2021},
month = {06},
pages = {},
title = {Attitude control of a fast-retargeting agile nanosatellite using Neural Network based steering for Variable Speed Control Moment Gyroscopes}
}
```

2. Design and Manufacturing of Hardware in Loop simulation testbed equipped with Variable Speed Control Moment Gyroscopes

```bibtex
@inproceedings{inproceedings,
author = {Deore, Siddharth and Santoni, Fabio and Piergentili, Fabrizio and Marzioli, Paolo},
year = {2021},
month = {06},
pages = {},
title = {Design and Manufacturing of Hardware in Loop simulation testbed equipped with Variable Speed Control Moment Gyroscopes}
}
```

## Maintainers
This repository is maintained by:
|<img src="https://github.com/siddharthdeore.png" width="32">| [siddharth deore](https://github.com/siddharthdeore)|
|--|--|
