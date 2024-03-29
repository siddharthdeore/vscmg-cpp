FROM ubuntu:focal
LABEL Name=vscmgcpp Version=0.0.1

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# These commands copy your files into the specified directory in the image
# and set that as the working location
COPY . /usr/src/vscmgcpp
WORKDIR /usr/src/vscmgcpp


# Update apt database and install reqired packages
RUN apt-get update && apt-get install -y            \
    # g++ build essential
    build-essential                                 \
    # cmake and ccmake
    cmake cmake-curses-gui                          \
    # Eigen3 for algebra
    libeigen3-dev                                   \
    # boost for ode integrator and python wrapper
    libboost-all-dev                                \
    # python3 for python bindings
    python3                                         \
    # pip
    python3-pip                                     \
    python-is-python3                               \
    && rm -rf /var/lib/apt/lists/*

# install numpy
RUN pip3 install numpy

# Set the locale
# RUN locale-gen en_US.UTF-8
# build annd install adcs library
RUN set -ex;                            \
    mkdir -p /usr/src/vscmgcpp/build;   \
    cd /usr/src/vscmgcpp/build;         \
    cmake .. -DBUILD_PYTHON_LIB=True;   \
    make;                               \
    make install

WORKDIR /usr/src/vscmgcpp/
RUN touch README.md
RUN python setup.py build
RUN pip install .

# Remove source code
RUN rm -rf cmake/ docker include/ src/ Dockerfile adcs.egg-info/

WORKDIR /usr/src/vscmgcpp/examples/python

CMD ["bash"]

