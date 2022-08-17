FROM ubuntu:latest

LABEL Name=vscmgcpp Version=0.0.1

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
# Set environment variables used by build
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib"

# Update apt database
RUN apt-get update

# Install baseline required tools
RUN apt install -y --no-install-recommends \
    # C++
    build-essential \
    #cmake
    cmake  cmake-curses-gui \
    # Eigen 3 and boost
    libeigen3-dev libboost-all-dev \
    # python and pip
    python3 python3-pip \
    # Required to run apt-get in the container
    sudo \
    # Do this cleanup every time to ensure minimal layer sizes
    && apt-get clean autoclean \
    && apt-get autoremove -y \
    && rm -rf /var/lib/{apt,dpkg,cache,log}


RUN pip3 install numpy



# These commands copy your files into the specified directory in the image
# and set that as the working location
COPY . /usr/src/vscmg-cpp
WORKDIR /usr/src/vscmg-cpp/

RUN rm /usr/src/vscmg-cpp/build /usr/src/vscmg-cpp/bin -rf
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_LIB=True
WORKDIR /usr/src/vscmg-cpp/build
RUN make -j$(nproc)
WORKDIR /usr/src/vscmg-cpp/bin

# This command runs your application, comment out this line to compile only
CMD ["bash"]

# to build dockerfile
# docker build --pull --rm -f "Dockerfile" -t vscmgcpp:latest "." 
# to run container
# docker run --rm -it  vscmgcpp:latest 
