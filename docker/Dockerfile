# Multi-stage Dockerfile for ROS2 Build Tool
# Production-grade with minimal attack surface

# Stage 1: Base ROS2 environment
FROM osrf/ros:humble-desktop AS ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-venv \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies management
RUN pip3 install --no-cache-dir \
    pipenv \
    poetry \
    pip-tools

# Stage 2: Development environment
FROM ros-base AS development

# Install development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    valgrind \
    vim \
    nano \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace

# Copy requirements
COPY requirements/ ./requirements/

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    -r requirements/base.txt \
    -r requirements/dev.txt \
    -r requirements/test.txt

# Stage 3: Testing environment
FROM development AS testing

# Install additional testing tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    lcov \
    gcovr \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
COPY src/ ./src/
COPY tests/ ./tests/
COPY pyproject.toml ./
COPY setup.cfg ./
COPY pytest.ini ./

# Run tests
RUN pytest tests/unit --cov=src/ros2_build_tool --cov-report=term-missing

# Stage 4: Production runtime
FROM ros-base AS production

# Create non-root user
RUN useradd -m -s /bin/bash ros2user && \
    echo "ros2user:ros2user" | chpasswd && \
    usermod -aG sudo ros2user

# Install only production dependencies
WORKDIR /opt/ros2_build_tool

# Copy requirements
COPY requirements/base.txt ./requirements.txt

# Install production dependencies
RUN pip3 install --no-cache-dir -r requirements.txt && \
    rm requirements.txt

# Copy only necessary files
COPY --from=testing /workspace/src ./src

# Set ownership
RUN chown -R ros2user:ros2user /opt/ros2_build_tool

# Switch to non-root user
USER ros2user

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Entry point
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["python3", "-m", "ros2_build_tool"]

# Stage 5: Minimal runtime (distroless)
FROM gcr.io/distroless/python3-debian11 AS minimal

# Copy Python environment and application
COPY --from=production /opt/ros2_build_tool /opt/ros2_build_tool
COPY --from=production /usr/local/lib/python3.10/dist-packages /usr/local/lib/python3.10/dist-packages

# Set working directory
WORKDIR /opt/ros2_build_tool

# Run as non-root
USER 1000

# Entry point
ENTRYPOINT ["python3", "-m", "ros2_build_tool"]