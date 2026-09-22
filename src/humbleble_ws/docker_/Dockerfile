# ベース: ROS 2 Humble + GUI(RViz)等を使える desktop イメージ
FROM osrf/ros:humble-desktop


# 非rootユーザを任意のUID/GIDで作成（ホストとのファイル権限ずれを防ぐ）
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000


# 開発に必要な定番ツール
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
bash-completion build-essential git curl wget nano vim \
python3-colcon-common-extensions python3-vcstool \
locales tzdata \
&& rm -rf /var/lib/apt/lists/*


# ロケール
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8


# ユーザ作成
RUN groupadd -g ${GID} ${USERNAME} \
&& useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
&& usermod -aG sudo ${USERNAME} || true \
&& echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" >/etc/sudoers.d/${USERNAME}

# devユーザのbashrcにカラーを有効化
RUN test -f /home/${USERNAME}/.bashrc || cp /etc/skel/.bashrc /home/${USERNAME}/.bashrc && \
    sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc && \
    grep -q 'ls --color=auto' /home/${USERNAME}/.bashrc || echo 'alias ls="ls --color=auto"' >> /home/${USERNAME}/.bashrc && \
    chown ${UID}:${GID} /home/${USERNAME}/.bashrc


# ROSセットアップ（便利なエイリアスなど）
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
&& echo "alias cw='cd ~/ws'" >> /etc/bash.bashrc \
&& echo "alias cs='cd ~/ws/src'" >> /etc/bash.bashrc \
&& echo "alias cb='cd ~/ws && colcon build --symlink-install'" >> /etc/bash.bashrc \
&& echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /etc/bash.bashrc \
&& echo "alias cn='cd ~/nyankichi_ws'" >> /etc/bash.bashrc \
&& echo "alias cbn='cd ~/nyankichi_ws && colcon build --symlink-install'" >> /etc/bash.bashrc


# --- nyankichi_ws 用追加設定 ---
# Ignition Fortress (gz_ros2_control) のために OSRF apt リポジトリを追加
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# nyankichi_ws の依存パッケージ
# gz_ros2_control: ignition-fortress, ros-gz-bridge/sim, ros2_control/controllers
# humbleble_ws:    navigation2, slam_toolbox, BehaviorTree.CPP, MoveIt2, OpenCV
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ignition-fortress \
    gz-tools2 \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-mouse-teleop \
    ros-humble-octomap \
    ros-humble-octomap-msgs \
    ros-humble-gazebo-msgs \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-ros-warehouse \
    ros-humble-moveit-setup-assistant \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers \
    ros-humble-tf2-eigen \
    ros-humble-image-pipeline \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*


# 作業ディレクトリを用意
USER ${USERNAME}
WORKDIR /home/${USERNAME}
RUN mkdir -p /home/${USERNAME}/ws/src


# エントリ
CMD ["bash"]
