#!/usr/bin/env bash
# Runs INSIDE the ros:noetic packaging container. Installs prebuilt PAIRS
# dependency .debs from /prebuilt, then builds pairs_octomap_server and copies the .deb to /output.
#   PAIRS deps (build order, supplied from /prebuilt): pairs_lib pairs_msgs pairs_octomap_tools
set -eo pipefail
ROS_DISTRO_NAME="noetic"; OS_NAME="ubuntu"; OS_VERSION="focal"
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

LOCAL_RULES="/src/pairs_octomap_server/packaging/rosdep/pairs.yaml"
echo "yaml file://${LOCAL_RULES}" | sudo tee /etc/ros/rosdep/sources.list.d/10-pairs.list >/dev/null
rosdep update --include-eol-distros
sudo apt-get update
mkdir -p /output

# Install prebuilt PAIRS dependency .debs (msgs/lib/hw_api/managers/...) from the pool.
shopt -s nullglob
prebuilt=(/prebuilt/ros-${ROS_DISTRO_NAME}-*.deb)
if [ ${#prebuilt[@]} -gt 0 ]; then
  echo ">> installing ${#prebuilt[@]} prebuilt PAIRS deps from /prebuilt"
  apt-get install -y "${prebuilt[@]}" || { dpkg -i "${prebuilt[@]}" || true; apt-get install -f -y || true; }
fi

build_one() {
  local pkg_dir="$1"; local pkg_name; pkg_name="$(basename "$pkg_dir")"
  if [ ! -d "$pkg_dir" ]; then echo "ERROR: $pkg_name not found at $pkg_dir" >&2; exit 1; fi
  echo "=================================================================="
  echo " Building Debian package for: ${pkg_name}"
  echo "=================================================================="
  local work="/ws/${pkg_name}"
  rm -rf "$work"; cp -a "$pkg_dir" "$work"; cd "$work"; rm -rf debian obj-*
  rosdep install --from-paths . --ignore-src -r -y || true
  bloom-generate rosdebian --os-name "$OS_NAME" --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO_NAME"
  # ros-<distro>-nlopt declares cmake version 2.4.2 but the apt package is 2.1.21,
  # so catkin emits an unsatisfiable "(>= 2.4.2)" — strip the version constraint.
  [ -f debian/control ] && sed -i -E 's/(ros-[a-z0-9]+-nlopt) \(>= [0-9.]+\)/\1/g' debian/control
  fakeroot debian/rules binary
  local deb
  for deb in /ws/ros-${ROS_DISTRO_NAME}-*"${pkg_name//_/-}"*.deb ../ros-${ROS_DISTRO_NAME}-*.deb; do
    [ -f "$deb" ] || continue
    cp -v "$deb" /output/
    apt-get install -y "$deb" || dpkg -i "$deb" || true
  done
}

build_one /src/pairs_octomap_server

echo "=================================================================="
echo " Done. Built packages in /output:"
ls -1 /output/*.deb 2>/dev/null || echo " (none)"
echo "=================================================================="
