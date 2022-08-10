# ADCS Toolbox

<img src="https://upload.wikimedia.org/wikipedia/commons/1/18/ISO_C%2B%2B_Logo.svg" width = "16" alt="C++ 14"> <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/1/13/Cmake.svg/900px-Cmake.svg.png" width = "16" alt="CMake"> <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b0/NewTux.svg/800px-NewTux.svg.png" width ="16" alt="Linux">
<img src="https://upload.wikimedia.org/wikipedia/commons/d/d1/Eigen_Silly_Professor_135x135.png" width ="16" alt="Eigen (C++ library)">
<img src="https://upload.wikimedia.org/wikipedia/commons/c/cd/Boost.png" width ="40" alt="BOOST (C++ library)">

High performance <i>Spacecraft Attitude Dynamics and Control System</i> toolbox with exposed python extention. Scalable toolbox uses Eigen3 for linear algebra and Boost for integration of system ODE . Currently there are three sample systems and more can be added provided that new systems are inherited from `IBaseSystem` base interface class and follow same structure. The <b>`pyadcs`</b> extention expose C++ classes and functions of <b>`ADCS`</b> library to fo.

# Systems
- Rigid Body - Simple free floating rigid body.
- VSCMG - Variable Speed Control Moment Gyroscope
- ~~RW4 - Satellite with 4 Reaction wheels~~

# Build
```
mkdir build && cd build
make
```

# Examples
Executable targets are located in `bin` directory
```
# Rigid Body 
cd ../bin
./TestRigidBody

# Variable Speed Control Moment Gyroscope
cd ../bin
./TestVSCMG

```
# Python Interface
Build with python interface wrapper library. Following comand executed in build directory shall build python interface library `pyadcs.so`  in `bin` directory on linux or `pyadcs.dll` on windows.
```console
cmake .. -DBUILD_PYTHON_LIB=True
make
```
# Run python samples
navigate to `bin` directory and run sample scripts `rigid_body.py` or `vscmg.py` eg.
```
python rigid_body.py
```

## Requirements
- C++ 14
- [Boost](https://www.boost.org/)
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- Python 2.7+



⚠️ Note : This toolbox was written for MSc Thesis titled <i> Neural Network based steering and Hardware in Loop Simulation of Variable Speed Control Moment Gyroscope</i>