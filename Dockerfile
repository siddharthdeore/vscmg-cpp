FROM ubuntu:latest

LABEL Name=vscmgcpp Version=0.0.1
# Install baseline required tools
RUN apt-get update && \
    apt install -y --no-install-recommends \
    # C++
    build-essential g++ gcc git make curl \
    # boost all dependencies
    libboost-all-dev netcat python3 python3-pip \
    #cmake
    cmake \
    # Eigen 3
    libeigen3-dev \
    # Required to run apt-get in the container
    sudo \
 # Do this cleanup every time to ensure minimal layer sizes
 && apt-get clean autoclean \
 && apt-get autoremove -y \
 && rm -rf /var/lib/{apt,dpkg,cache,log}


# Non-layer configuration
# Set environment variables used by build
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib"

RUN pip3 install numpy

# These commands copy your files into the specified directory in the image
# and set that as the working location
COPY . /usr/src/vscmg-cpp
WORKDIR /usr/src/vscmg-cpp

# This command compiles your app using GCC, adjust for your source code

# Run cmake configure & build process
RUN cd /usr/src/vscmg-cpp
RUN cmake -B/build -S . -D CMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_LIB=True
RUN cmake --build /build

# This command runs your application, comment out this line to compile only
CMD bash
EXPOSE 8888

