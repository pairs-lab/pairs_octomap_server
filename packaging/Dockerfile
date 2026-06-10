# Toolchain image for building ROS jazzy .deb packages for pairs_octomap_server.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-bloom \
      python3-rosdep \
      python3-catkin-pkg \
      fakeroot \
      debhelper \
      dh-python \
      git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY build_deb.sh /usr/local/bin/build_deb.sh
RUN chmod +x /usr/local/bin/build_deb.sh

ENTRYPOINT ["/usr/local/bin/build_deb.sh"]
