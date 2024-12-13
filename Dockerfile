#!/usr/bin/env docker

# This is a template for a Dockerfile to build a docker image for your ROS package. 
# The main purpose of this file is to install dependencies for your package.

# FROM ros:noetic-ros-base-focal        
FROM ros:melodic-ros-base-bionic      
####<--- TODO: change to your base image

# ENV ROS_ROOT=/opt/ros/noetic        
ENV ROS_ROOT=/opt/ros/melodic         
#<--- TODO: change to your ROS version to match base image

# Set up workspace
RUN mkdir -p /ws/src   

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Package apt dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    python3-catkin-tools \
    python-pip \
    libblas-dev \
    liblapack-dev \
    ros-melodic-tf2 \
    ros-melodic-tf \
    # EXAMPLE: \
    # build-essential \
    # libssl-dev \
    # libffi-dev \
    # python3-setuptools \
    # python3-venv \
    # python3-tk \
    gfortran \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip install scipy

RUN pip install \
    osqp==0.6.1 \
    -U numpy
RUN pip install --upgrade --force-reinstall numpy
RUN pip list

# Download and install Intel MKL
RUN wget https://anaconda.org/intel/mkl/2023.0.0/download/linux-64/mkl-2023.0.0-intel64.tar.bz2 \
    && tar -xjf mkl-2023.0.0-intel64.tar.bz2 \
    && rm mkl-2023.0.0-intel64.tar.bz2

# Set environment variables for MKL
ENV MKLROOT=/mkl-2023.0.0-intel64

# Install Pardiso solver
RUN cd /mkl-2023.0.0-intel64 \
    && ./configure \
    && make \
    && make install

# Installing of pip dependencies
# RUN pip3 install \
#     # EXAMPLE: \
#     # torch \
#     # torchvision \
#     # tensorboardX \
#     # opencv-python \
#     # scikit-image \
#     # scikit-learn \


# Optional: Install additional dependencies with script
# COPY scripts/install.sh scripts/
# RUN chmod +x scripts/install.sh && bash scripts/install.sh

WORKDIR /ws
