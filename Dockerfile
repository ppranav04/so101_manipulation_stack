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
COPY so101_hardware/package.xml src/so101_manipulation_stack/so101_hardware/
COPY so101_bringup/package.xml src/so101_manipulation_stack/so101_bringup/
COPY so101_description/package.xml src/so101_manipulation_stack/so101_description/
COPY so101_calibration/package.xml src/so101_manipulation_stack/so101_calibration/
COPY so101_ik/package.xml src/so101_manipulation_stack/so101_ik/
COPY so101_perception/package.xml src/so101_manipulation_stack/so101_perception/

# Install everything declared in the package.xml files.
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

COPY . src/so101_manipulation_stack

# Build, test, and fail the image build if any test fails.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
    && colcon test \
    && colcon test-result --verbose

# Running the image drops you into a shell with the workspace sourced.
RUN echo "source /so101_ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]
