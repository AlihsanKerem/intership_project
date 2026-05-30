# Temel imaj olarak ROS 2 Humble Desktop kullanıyoruz (RViz vb. arayüz araçları hazır gelir)
FROM osrf/ros:humble-desktop

# Gerekli sistem paketlerini ve Gazebo araçlarını kuralım
RUN apt-get update && apt-get install -y \
    nano \
    git \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Container içinde çalışacağımız ana dizini belirliyoruz
WORKDIR /project_ws

# Her terminal açıldığında ROS 2 ortamının otomatik yüklenmesi için bashrc'ye ekleme yapıyoruz
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /project_ws/install/setup.bash ]; then source /project_ws/install/setup.bash; fi" >> ~/.bashrc

# Sistemi açık tutmak için varsayılan bir komut belirliyoruz
CMD ["/bin/bash"]