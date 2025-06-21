# Multi-architecture Dockerfile for VLM Workspace
# Supports both AMD64 (development) and ARM64 (deployment)

ARG TARGETARCH
ARG BUILDPLATFORM

# Choose base image based on target architecture
FROM 192.168.31.199:8000/library/ros:humble_${TARGETARCH} AS base

# Build arguments
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=developer
ARG BUILD_TYPE=development
ARG TARGETARCH

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    TZ=Asia/Shanghai \
    ROS_DISTRO=humble

# Copy architecture-specific sources list
COPY .devcontainer/sources_${TARGETARCH}.list /etc/apt/sources.list

# Install common dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        wget \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        libpython3-dev \
        ros-humble-common-interfaces \
        ros-humble-sensor-msgs-py \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-msgs \
        ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install development-specific packages (only for development builds)
RUN if [ "$BUILD_TYPE" = "development" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            python3-rosdep \
            python3-vcstool \
            python3-setuptools \
            lsb-release \
            sudo \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Configure pip and install Python packages (as root for both cases)
RUN pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple && \
    pip install \
        empy==3.3.4 \
        colcon-common-extensions \
        numpy==1.26.4 \
        scipy \
        httpx \
        requests \
        opencv-python \
        av \
        lark \ 
        deep_translator \ 
        litellm \
        fastapi \
        tqdm \
        pydantic \
        pillow \
        uvicorn

# Install additional development packages
RUN if [ "$BUILD_TYPE" = "development" ]; then \
        pip install pytest-asyncio; \
    fi

# Create user for development builds only
RUN if [ "$BUILD_TYPE" = "development" ]; then \
        if [ -z "${USER_ID}" ] || [ -z "${GROUP_ID}" ] || [ -z "${USER_NAME}" ]; then \
            echo "Error: USER_ID, GROUP_ID, and USER_NAME must be set for development builds." && exit 1; \
        fi && \
        groupadd -g ${GROUP_ID} ${USER_NAME} 2>/dev/null || true && \
        useradd --create-home --shell /bin/bash -u ${USER_ID} -g ${GROUP_ID} ${USER_NAME} && \
        echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} && \
        chmod 0440 /etc/sudoers.d/${USER_NAME}; \
    fi

# Setup .bashrc with ROS environment
RUN if [ "$BUILD_TYPE" = "development" ]; then \
        echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc && \
        echo "source /home/heng.li/repo/vlm_ws/install/setup.bash" >> /home/${USER_NAME}/.bashrc && \
        chown ${USER_NAME}:${USER_NAME} /home/${USER_NAME}/.bashrc; \
    else \
        echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc; \
    fi

# Install x-cmd for development builds only
RUN if [ "$BUILD_TYPE" = "development" ]; then \
        curl -fsSL https://get.x-cmd.com | bash; \
    fi

# Create startup script that handles user switching and working directory
RUN echo '#!/bin/bash' > /startup.sh && \
    echo 'if [ "$BUILD_TYPE" = "development" ]; then' >> /startup.sh && \
    echo '    cd /home/heng.li/repo/vlm_ws' >> /startup.sh && \
    echo '    if [ "$(id -u)" = "0" ]; then' >> /startup.sh && \
    echo '        exec su - '"${USER_NAME}"' -c "cd /home/heng.li/repo/vlm_ws && exec \"\$@\""' >> /startup.sh && \
    echo '    else' >> /startup.sh && \
    echo '        exec "$@"' >> /startup.sh && \
    echo '    fi' >> /startup.sh && \
    echo 'else' >> /startup.sh && \
    echo '    cd /vlm_ws' >> /startup.sh && \
    echo '    exec "$@"' >> /startup.sh && \
    echo 'fi' >> /startup.sh && \
    chmod +x /startup.sh

# Set working directory (will be overridden by startup script)
WORKDIR /

# Add build info
RUN echo "Built for architecture: ${TARGETARCH}" > /tmp/build-info.txt && \
    echo "Build type: ${BUILD_TYPE}" >> /tmp/build-info.txt

# Use startup script as entrypoint
ENTRYPOINT ["/startup.sh"]

# Default command
CMD ["/bin/bash"]
