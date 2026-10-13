# Reproducible build-and-test image for so101_manipulation_stack.
# Usage (from the repo root):
#   docker build -t so101_manipulation_stack .
# If the build finishes, every package built and every test passed.

FROM ros:jazzy

# Use bash so `source` works in RUN lines.
SHELL ["/bin/bash", "-c"]

# Same setting as CI: run cppcheck despite the slow-version guard.
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# Workspace layout matches the laptop and CI: <ws>/src/<repo>
WORKDIR /so101_ws
COPY . src/so101_manipulation_stack

# Install everything declared in the package.xml files.
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# Build, test, and fail the image build if any test fails.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
    && colcon test \
    && colcon test-result --verbose

# Running the image drops you into a shell with the workspace sourced.
RUN echo "source /so101_ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]
