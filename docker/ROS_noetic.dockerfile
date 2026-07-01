# Use an official ROS Noetic full desktop image as the base
FROM osrf/ros:noetic-desktop-full

# Avoid prompts from APT during build by specifying non-interactive as the frontend
ARG DEBIAN_FRONTEND=noninteractive
# Set the timezone environment variable
ENV TZ=America/Sao_Paulo
# Set the location the Gazebo Plugin Path
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}

# Define user-related arguments to create a non-root user inside the container
ARG USERNAME=tommaselli
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a new group and user with the specified UID and GID, create a config directory, and set ownership
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Update the package list, install sudo, configure sudoers for the new user without password prompts, and clean up APT lists
RUN apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install git
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential \
       cmake \
       pkg-config \
       git \
       curl \
       wget \
       ca-certificates \
       gnupg2 \
       nano \
       python3 \
       python3-pip \
       python3-catkin-tools \
       ros-noetic-gazebo-dev \
       ros-noetic-hector-gazebo-plugins \
       ros-noetic-cv-bridge \
       libgl1-mesa-glx \
       libgl1-mesa-dri \
       mesa-utils \
       ffmpeg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    . /opt/conda/etc/profile.d/conda.sh && \
    conda init && \
    conda clean -ya
ENV PATH=/opt/conda/bin:$PATH
SHELL ["/bin/bash", "-c"]

# Fix ROS/conda environment conflicts - prioritize system libraries for ROS
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/noetic/lib
ENV PYTHONPATH=""
# Fix libffi/cv_bridge conflict
RUN apt-get update && apt-get install -y --reinstall libffi7 libffi-dev && rm -rf /var/lib/apt/lists/*

# conda environment
COPY docker/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json
COPY docker/environment.yaml /root
RUN conda tos accept --override-channels --channel defaults && \
    conda update conda && \
    conda env update -n base -f /root/environment.yaml && \
    rm /root/environment.yaml && \
    conda clean -ya && \
    pip cache purge

# success!
RUN echo "Successfully built ROS TD-MPC2 Docker image!"

# Multicolor bash prompt for root + color terminal
RUN echo 'export PS1="\[\e[1;32m\]\u\[\e[0m\]@\[\e[1;34m\]\h\[\e[0m\]:\[\e[1;33m\]\w\[\e[0m\]\$ "' >> /root/.bashrc
RUN echo "export TERM=xterm-256color" >> /etc/bash.bashrc

# GL/NVIDIA rendering hints
RUN echo 'export SVGA_VGPU10=0 \
export LIBGL_ALWAYS_SOFTWARE=0 \
export LIBGL_DEBUG=verbose \
export LD_LIBRARY_PATH=/usr/lib/nvidia-535:$LD_LIBRARY_PATH \
' >> /root/.bashrc

# Custom entrypoint + user bashrc
COPY docker/config/entrypoint.sh /entrypoint.sh
COPY docker/config/bashrc /home/${USERNAME}/.bashrc
RUN chown $USER_UID:$USER_GID /home/$USERNAME/.bashrc

# Normalize ROS folder ownership/permissions
RUN chown -R root:root /opt/ros/noetic \
    && chmod -R 755 /opt/ros/noetic

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
CMD [ "bash" ]
