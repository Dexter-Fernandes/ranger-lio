# ranger-lio development image.
# Ubuntu 24.04 + ROS 2 Jazzy, with GTSAM and small_gicp built from pinned tags.
# Build:  docker build -f .devcontainer/Dockerfile -t ranger-lio:dev .
# The GTSAM stage takes roughly an hour on a 2-core laptop; layers are ordered so it caches.

FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG JOBS=4

# ---- system packages -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git curl ca-certificates sudo \
        libeigen3-dev libboost-all-dev libtbb-dev libomp-dev \
        python3 python3-pip python3-venv \
        ros-jazzy-rosbag2-cpp ros-jazzy-rosbag2-storage-mcap ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---- GTSAM (pinned) ----------------------------------------------------------
# System Eigen is mandatory: small_gicp and GTSAM must share one Eigen.
ARG GTSAM_VERSION=4.2.2
RUN git clone --depth 1 --branch ${GTSAM_VERSION} https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/gtsam/build -j${JOBS} \
    && cmake --install /tmp/gtsam/build \
    && rm -rf /tmp/gtsam

# ---- small_gicp (pinned) -----------------------------------------------------
ARG SMALL_GICP_VERSION=v1.0.1
RUN git clone --depth 1 --branch ${SMALL_GICP_VERSION} https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_HELPER=ON \
        -DBUILD_WITH_OPENMP=ON \
        -DBUILD_WITH_TBB=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTS=OFF \
    && cmake --build /tmp/small_gicp/build -j${JOBS} \
    && cmake --install /tmp/small_gicp/build \
    && rm -rf /tmp/small_gicp \
    && ldconfig

# ---- GoogleTest source (pinned), consumed by FetchContent without a download ----
ARG GTEST_VERSION=v1.18.0
RUN git clone --depth 1 --branch ${GTEST_VERSION} https://github.com/google/googletest.git /opt/deps/googletest
ENV RANGER_GTEST_SRC=/opt/deps/googletest

# ---- Python tooling: bag conversion and evaluation ----------------------------
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir \
        rosbags==0.11.5 \
        evo==1.37.1 \
        numpy scipy pyyaml
ENV PATH=/opt/venv/bin:${PATH}
ENV MPLBACKEND=Agg

# ---- non-root user matching the host UID so bind-mounted files stay owned by the host user ----
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN (id -u ${USER_UID} >/dev/null 2>&1 && userdel -r "$(id -un ${USER_UID})" || true) \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspaces/ranger-lio
CMD ["/bin/bash"]
