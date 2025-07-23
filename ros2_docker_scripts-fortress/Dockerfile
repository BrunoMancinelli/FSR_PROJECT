FROM osrf/ros:humble-desktop

# Install essential packages and Python3 pip
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl

# Scarica e installa pip manualmente se necessario
RUN curl -sS https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py

# Aggiungi la directory di pip al PATH
ENV PATH=$PATH:/home/user/.local/bin

# Disinstallare eventuali versioni di opencv installate e installare quelle specificate
RUN pip uninstall opencv-python opencv-contrib-python -y && \
    pip install opencv-python==3.4.18.65 && \
    pip install opencv-contrib-python==3.4.18.65

# Altri comandi già presenti nel Dockerfile
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-gz \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-ign-ros2-control-demos \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-usb-cam \
    ros-humble-image-pipeline \
    ros-humble-tf-transformations \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    python3-pip

# Ambiente e variabili di ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME=/home/user
ENV ROS_DISTRO=humble

# Creazione utente non root
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers
USER user

# Configurazione e costruzione dello spazio di lavoro ROS2
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep update; rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y; colcon build --symlink-install

# Aggiungere comandi al .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ${HOME}/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ${HOME}/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${HOME}/.bashrc

USER root
RUN apt-get upgrade -y && apt-get update -y

RUN apt-get install ros-humble-ros-ign-bridge -y
RUN apt-get install ros-humble-usb-cam -y
RUN apt-get install ros-humble-image-pipeline -y
RUN apt-get install ros-humble-tf-transformations -y


RUN apt-get update && apt-get install -y libgz-sim6-plugins
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz/sim6/models:${GZ_SIM_RESOURCE_PATH}



# Pulizia dell'immagine
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user

