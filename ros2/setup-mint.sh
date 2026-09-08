#!/usr/bin/env bash
#
# ROS 2 Jazzy setup for Linux Mint 22.x (Ubuntu 24.04 "noble" base).
#
# THE MINT GOTCHA THIS SCRIPT EXISTS FOR
#   Every ROS 2 install guide says to use `lsb_release -cs` for the apt source
#   codename. On Mint that returns "xia" (or "wilma", "vera"...), which is a
#   Mint release name that no ROS repository has ever heard of. apt then fails
#   with a 404 that reads like the repo is down. Mint publishes its Ubuntu base
#   separately in /etc/upstream-release/lsb-release, and that is what must be
#   used. This script reads it and refuses to continue if it is not "noble".
#
# WHAT IT INSTALLS
#   ROS 2 Jazzy Jalisco (desktop: includes RViz2), dev tools, colcon.
#   Isaac Sim and MATLAB are separate -- see ros2/README.md.
#
# Safe to re-run.

set -euo pipefail

GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RED=$'\033[31m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
step() { echo; echo "${BOLD}==> $*${OFF}"; }
ok()   { echo "  ${GREEN}ok${OFF}   $*"; }
warn() { echo "  ${YELLOW}warn${OFF} $*"; }
die()  { echo "  ${RED}FAIL${OFF} $*" >&2; exit 1; }

ROS_DISTRO_WANTED=jazzy

# ---------------------------------------------------------------------------
step "1. Checking the Ubuntu base underneath Mint"
# ---------------------------------------------------------------------------

[ -f /etc/upstream-release/lsb-release ] \
  && UBUNTU_CODENAME=$(. /etc/upstream-release/lsb-release && echo "$DISTRIB_CODENAME") \
  || UBUNTU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")

MINT_NAME=$(. /etc/os-release && echo "$PRETTY_NAME")
echo "  distro:       $MINT_NAME"
echo "  ubuntu base:  ${UBUNTU_CODENAME:-UNKNOWN}"
echo "  lsb_release:  $(lsb_release -cs 2>/dev/null || echo n/a)   <- NOT usable for apt"

[ "${UBUNTU_CODENAME:-}" = "noble" ] || die \
  "expected an Ubuntu 24.04 'noble' base (Mint 22.x). Got '${UBUNTU_CODENAME:-unknown}'.
       ROS 2 Jazzy targets noble. On a Mint 21.x (jammy) base you want Humble
       instead -- rerun with ROS_DISTRO_WANTED=humble, and expect Isaac Sim to
       be happier on 24.04 regardless."
ok "noble base confirmed -- ROS 2 $ROS_DISTRO_WANTED is the right distro"

# ---------------------------------------------------------------------------
step "2. Locale (ROS 2 needs a UTF-8 locale)"
# ---------------------------------------------------------------------------

if locale | grep -qi 'utf-8'; then
  ok "UTF-8 locale already set"
else
  sudo apt-get update -qq
  sudo apt-get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  ok "UTF-8 locale generated"
fi

# ---------------------------------------------------------------------------
step "3. Enabling the universe repository"
# ---------------------------------------------------------------------------

sudo apt-get install -y software-properties-common curl >/dev/null
sudo add-apt-repository -y universe >/dev/null 2>&1 || true
ok "universe enabled"

# ---------------------------------------------------------------------------
step "4. Adding the ROS 2 apt source"
# ---------------------------------------------------------------------------

# ROS now ships its apt source as a .deb rather than a hand-written list file.
# It pins the codename itself, which is why UBUNTU_CODENAME is exported here --
# the package reads the environment rather than calling lsb_release.
ROS_APT_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                  | grep -oP '"tag_name": "\K[^"]+' || true)

if [ -z "$ROS_APT_VERSION" ]; then
  warn "could not reach GitHub for the ros-apt-source version; falling back to the manual keyring"
  sudo curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  ok "apt source written manually, pinned to ${UBUNTU_CODENAME}"
else
  TMP=$(mktemp -d)
  curl -fsSL -o "$TMP/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.${UBUNTU_CODENAME}_all.deb"
  sudo dpkg -i "$TMP/ros2-apt-source.deb"
  rm -rf "$TMP"
  ok "ros2-apt-source ${ROS_APT_VERSION} installed for ${UBUNTU_CODENAME}"
fi

# ---------------------------------------------------------------------------
step "5. Installing ROS 2 $ROS_DISTRO_WANTED desktop"
# ---------------------------------------------------------------------------

sudo apt-get update
sudo apt-get upgrade -y
echo "  this is a few GB and will take a while..."
sudo apt-get install -y \
  "ros-${ROS_DISTRO_WANTED}-desktop" \
  ros-dev-tools \
  "ros-${ROS_DISTRO_WANTED}-vision-msgs" \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial
ok "ROS 2 ${ROS_DISTRO_WANTED} desktop installed (RViz2 included)"

# ---------------------------------------------------------------------------
step "6. rosdep"
# ---------------------------------------------------------------------------

sudo rosdep init 2>/dev/null || true
rosdep update
ok "rosdep ready"

# ---------------------------------------------------------------------------
step "7. Shell setup"
# ---------------------------------------------------------------------------

LINE="source /opt/ros/${ROS_DISTRO_WANTED}/setup.bash"
if grep -qxF "$LINE" ~/.bashrc; then
  ok "~/.bashrc already sources ROS"
else
  {
    echo ""
    echo "# ROS 2 ${ROS_DISTRO_WANTED}"
    echo "$LINE"
    echo "export ROS_DOMAIN_ID=0   # every machine on this rig must match"
  } >> ~/.bashrc
  ok "added ROS sourcing to ~/.bashrc"
fi

# ---------------------------------------------------------------------------
step "8. Verifying"
# ---------------------------------------------------------------------------

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_WANTED}/setup.bash"
echo "  ROS_DISTRO   = ${ROS_DISTRO:-unset}"
echo "  ros2 version = $(ros2 --version 2>/dev/null || echo 'not found')"
command -v rviz2 >/dev/null && ok "rviz2 present" || warn "rviz2 missing"

echo
echo "${BOLD}Done.${OFF} Open a NEW terminal, then try:"
echo
echo "    ros2 run demo_nodes_cpp talker      # one terminal"
echo "    ros2 run demo_nodes_py listener     # another"
echo "    rviz2                               # should open a window"
echo
echo "Next: Isaac Sim and MATLAB -- see ros2/README.md"
