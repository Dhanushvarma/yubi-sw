#!/usr/bin/env bash
set -euo pipefail

# -----------------------
# Variant resolution: CLI flag overrides ROBOT_VARIANT env var; default
# stationary (matches docker-compose / launch defaults). 'portable' expects
# 3 USB cameras (same VID/PID); 'stationary' expects 2.
# -----------------------
VARIANT="${ROBOT_VARIANT:-stationary}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --variant)
      [[ $# -ge 2 ]] || { echo "ERROR: --variant requires an argument"; exit 2; }
      VARIANT="$2"
      shift 2
      ;;
    --variant=*)
      VARIANT="${1#--variant=}"
      shift
      ;;
    -h|--help)
      cat <<'USAGE'
Usage: sudo -E ./tools/yubi_udev_setup.sh [--variant stationary|portable|portable_head]

Calibrates the gripper encoders and writes /etc/udev/rules.d/99-yubi-devices.rules.

Variant selection (precedence: CLI flag > $ROBOT_VARIANT env var > stationary):
  stationary  expects 2 USB cameras (left, right)
  portable    expects 3 USB cameras (left, right, center/head) — the head
              camera is the same USB model as the hands, so we use the
              gripper L/R USB-hub topology to identify which is which.
USAGE
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1"; exit 2
      ;;
  esac
done

# portable_head has the same USB devices as portable (L/R/center cameras,
# two ESP32s); only the Quest app and the camera mount differ.
if [[ "$VARIANT" == "portable_head" ]]; then
  VARIANT=portable
fi

case "$VARIANT" in
  stationary) EXPECTED_CAM_COUNT=2 ;;
  portable)   EXPECTED_CAM_COUNT=3 ;;
  *)
    echo "ERROR: unsupported variant: $VARIANT (expected: stationary, portable)"
    exit 2
    ;;
esac
echo "Variant: $VARIANT (expect $EXPECTED_CAM_COUNT cameras)"

# -----------------------
# Pause the yubi container while we rewrite udev rules, so it doesn't
# hold /dev/video* handles (camera open fails if the container is up) and
# doesn't race on device symlinks. Covers both systemd-managed runs
# (docker.yubi.service) and manual `docker compose up` runs.
#
# We do NOT restart yubi on exit; bring it back up yourself once setup is
# done (e.g. `docker compose up -d`, or `systemctl start docker.yubi.service`).
# -----------------------
YUBI_SERVICE="docker.yubi.service"
YUBI_CONTAINER="yubi"

service_exists() {
  command -v systemctl >/dev/null 2>&1 \
    && systemctl list-unit-files --no-legend 2>/dev/null \
       | awk '{print $1}' | grep -qx "$YUBI_SERVICE"
}

container_is_running() {
  command -v docker >/dev/null 2>&1 || return 1
  [[ "$(docker inspect -f '{{.State.Running}}' "$YUBI_CONTAINER" 2>/dev/null)" == "true" ]]
}

if service_exists && systemctl is-active --quiet "$YUBI_SERVICE"; then
  echo "Stopping $YUBI_SERVICE for udev setup..."
  systemctl stop "$YUBI_SERVICE"
elif container_is_running; then
  echo "Stopping container $YUBI_CONTAINER for udev setup..."
  docker stop "$YUBI_CONTAINER" >/dev/null
fi

# -----------------------
# Camera (UVC) target
# -----------------------
CAM_VID="32e4"
CAM_PID="9230"
CAM_LEFT_SYMLINK="yubi_left_camera"
CAM_RIGHT_SYMLINK="yubi_right_camera"
CAM_CENTER_SYMLINK="yubi_center_camera"

# -----------------------
# ESP32C6 USB JTAG/serial (AS5048A over SPI -> streamed over USB serial)
# -----------------------
ESP_VID="303a"
ESP_PID="1001"
ESP_LEFT_SYMLINK="yubi_left_esp32c6"
ESP_RIGHT_SYMLINK="yubi_right_esp32c6"

# -----------------------
# Outputs
# -----------------------
CALIB_DIR="/etc/yubi"
CALIB_FILE="$CALIB_DIR/encoder_limits.yaml"
RULE_FILE="/etc/udev/rules.d/99-yubi-devices.rules"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

require() { command -v "$1" >/dev/null 2>&1 || { echo "Missing: $1"; exit 2; }; }
require udevadm
require python3

if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo:"
  echo "  sudo -E $0"
  exit 1
fi

# If running under sudo from a desktop session, try to keep GUI working.
if [[ -n "${SUDO_USER:-}" ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  if [[ -z "${XAUTHORITY:-}" ]]; then
    if [[ -f "/home/$SUDO_USER/.Xauthority" ]]; then
      export XAUTHORITY="/home/$SUDO_USER/.Xauthority"
    fi
  fi
fi

APT_UPDATED=0

apt_install() {
  local pkg="$1"
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    return 0
  fi
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "ERROR: apt-get not available to install $pkg" >&2
    return 1
  fi
  if [[ $APT_UPDATED -eq 0 ]]; then
    apt-get update
    APT_UPDATED=1
  fi
  DEBIAN_FRONTEND=noninteractive apt-get install -y "$pkg"
}

ensure_pip() {
  if python3 -m pip --version >/dev/null 2>&1; then
    return 0
  fi
  if command -v apt-get >/dev/null 2>&1; then
    echo "Installing python3-pip via apt..."
    apt_install python3-pip
    return 0
  fi
  echo "ERROR: pip is required but could not be installed automatically." >&2
  exit 2
}

ensure_python_dep() {
  local module="$1"
  local apt_pkg="${2:-}"
  local pip_pkg="${3:-$1}"
  if python3 -c "import $module" >/dev/null 2>&1; then
    return 0
  fi
  echo "Python module '$module' missing; attempting automatic installation..."
  if [[ -n "$apt_pkg" ]] && command -v apt-get >/dev/null 2>&1; then
    if apt_install "$apt_pkg"; then
      if python3 -c "import $module" >/dev/null 2>&1; then
        return 0
      fi
    fi
  fi
  ensure_pip
  if python3 -m pip install -U "$pip_pkg"; then
    if python3 -c "import $module" >/dev/null 2>&1; then
      return 0
    fi
  fi
  echo "ERROR: Failed to install Python module '$module'." >&2
  exit 2
}

# GUI deps
ensure_python_dep "tkinter" "python3-tk" "tk"
ensure_python_dep "serial" "python3-serial" "pyserial"
ensure_python_dep "cv2" "python3-opencv" "opencv-python"
ensure_python_dep "PIL.Image" "python3-pil" "pillow"
ensure_python_dep "PIL.ImageTk" "python3-pil.imagetk" "pillow"
ensure_python_dep "matplotlib" "python3-matplotlib" "matplotlib"

get_prop() {
  local info="$1"
  local key="$2"
  grep -E "^${key}=" <<<"$info" | head -n1 | cut -d= -f2- || true
}

# -----------------------
# Camera detection: /dev/video* with index==0, matching VID/PID
# -----------------------
cam_candidates=()
for dev in /dev/video*; do
  [[ -e "$dev" ]] || continue
  idx_path="/sys/class/video4linux/$(basename "$dev")/index"
  [[ -f "$idx_path" ]] || continue
  idx="$(cat "$idx_path")"
  [[ "$idx" == "0" ]] || continue

  info="$(udevadm info --query=property --name="$dev" 2>/dev/null || true)"
  v="$(get_prop "$info" "ID_VENDOR_ID")"
  p="$(get_prop "$info" "ID_MODEL_ID")"
  [[ "$v" == "$CAM_VID" && "$p" == "$CAM_PID" ]] || continue

  id_path="$(get_prop "$info" "ID_PATH")"
  if [[ -z "$id_path" ]]; then
    id_path="$(get_prop "$info" "ID_PATH_TAG")"
  fi
  [[ -n "$id_path" ]] || { echo "WARN: $dev has no ID_PATH(_TAG); skip"; continue; }

  cam_candidates+=("$dev|$id_path")
done

if [[ ${#cam_candidates[@]} -ne $EXPECTED_CAM_COUNT ]]; then
  echo "ERROR: variant=$VARIANT expects exactly $EXPECTED_CAM_COUNT cameras (VID:PID=$CAM_VID:$CAM_PID index=0), found ${#cam_candidates[@]}."
  printf '  %s\n' "${cam_candidates[@]:-}"
  exit 3
fi

cam1="${cam_candidates[0]%%|*}"
cam1_path="${cam_candidates[0]#*|}"
cam2="${cam_candidates[1]%%|*}"
cam2_path="${cam_candidates[1]#*|}"
cam3=""
cam3_path=""
if [[ "$VARIANT" == "portable" ]]; then
  cam3="${cam_candidates[2]%%|*}"
  cam3_path="${cam_candidates[2]#*|}"
fi

# -----------------------
# ESP32C6 detection: tty devices with VID/PID = 303a:1001
# Use pyserial list_ports in the GUI script (more robust),
# but we also sanity-check count here by asking the GUI to validate.
# -----------------------

echo "Detected candidates:"
echo "  Camera A: $cam1  ID_PATH=$cam1_path"
echo "  Camera B: $cam2  ID_PATH=$cam2_path"
if [[ -n "$cam3" ]]; then
  echo "  Camera C: $cam3  ID_PATH=$cam3_path"
fi
echo

# -----------------------
# Run unified GUI and get JSON result
# -----------------------
GUI="$SCRIPT_DIR/yubi_device_select_gui.py"
if [[ ! -f "$GUI" ]]; then
  echo "ERROR: GUI script not found: $GUI"
  exit 4
fi

echo "Launching GUI..."
GUI_ARGS=(
  --cam-vid "$CAM_VID" --cam-pid "$CAM_PID"
  --cam1 "$cam1" --cam2 "$cam2"
  --esp-vid "$ESP_VID" --esp-pid "$ESP_PID"
  --baud 115200
  --variant "$VARIANT"
)
if [[ -n "$cam3" ]]; then
  GUI_ARGS+=(--cam3 "$cam3")
fi
result_json="$(python3 "$GUI" "${GUI_ARGS[@]}")"

# Parse JSON in bash via python (robust)
cam_left="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["camera"]["left"]["dev"])
PY
)"
cam_right="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["camera"]["right"]["dev"])
PY
)"
cam_center=""
if [[ "$VARIANT" == "portable" ]]; then
  cam_center="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["camera"]["center"]["dev"])
PY
)"
fi

esp_left_serial="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["esp32"]["left"]["serial"])
PY
)"
esp_right_serial="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["esp32"]["right"]["serial"])
PY
)"

left_min="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["esp32"]["left"]["min"])
PY
)"
right_min="$(python3 - <<PY
import json
print(json.loads('''$result_json''')["esp32"]["right"]["min"])
PY
)"

# Map camera dev -> ID_PATH
declare -A CAM_PATH_BY_DEV
CAM_PATH_BY_DEV["$cam1"]="$cam1_path"
CAM_PATH_BY_DEV["$cam2"]="$cam2_path"
if [[ -n "$cam3" ]]; then
  CAM_PATH_BY_DEV["$cam3"]="$cam3_path"
fi
cam_left_path="${CAM_PATH_BY_DEV[$cam_left]}"
cam_right_path="${CAM_PATH_BY_DEV[$cam_right]}"
cam_center_path=""
if [[ -n "$cam_center" ]]; then
  cam_center_path="${CAM_PATH_BY_DEV[$cam_center]}"
fi

echo
echo "Assigned:"
echo "  Camera left  : $cam_left  ID_PATH=$cam_left_path"
echo "  Camera right : $cam_right ID_PATH=$cam_right_path"
if [[ -n "$cam_center" ]]; then
  echo "  Camera center: $cam_center ID_PATH=$cam_center_path"
fi
echo "  ESP32 left   : serial=$esp_left_serial  min=$left_min"
echo "  ESP32 right  : serial=$esp_right_serial min=$right_min"
echo

# -----------------------
# Write ROS param yaml (example structure)
# -----------------------
mkdir -p "$CALIB_DIR"
timestamp="$(date --iso-8601=seconds)"

cat > "$CALIB_FILE" <<EOF
# Auto-generated by yubi_udev_setup.sh on $timestamp
encoder_node:
  ros__parameters:
    left_min_raw: $left_min
    right_min_raw: $right_min
EOF

chmod 644 "$CALIB_FILE"
echo "Wrote: $CALIB_FILE"

# -----------------------
# Write udev rules directly into /etc/udev/rules.d
# -----------------------
if [[ -f "$RULE_FILE" ]]; then
  cp -a "$RULE_FILE" "${RULE_FILE}.$(date +%Y%m%d_%H%M%S).bak"
fi

cat > "$RULE_FILE" <<EOF
# Auto-generated by yubi_udev_setup.sh (variant=$VARIANT)
# Cameras (UVC): $CAM_VID:$CAM_PID index=0
SUBSYSTEM=="video4linux", KERNEL=="video*", ATTRS{idVendor}=="$CAM_VID", ATTRS{idProduct}=="$CAM_PID", ATTR{index}=="0", ENV{ID_PATH}=="$cam_left_path",  SYMLINK+="$CAM_LEFT_SYMLINK"
SUBSYSTEM=="video4linux", KERNEL=="video*", ATTRS{idVendor}=="$CAM_VID", ATTRS{idProduct}=="$CAM_PID", ATTR{index}=="0", ENV{ID_PATH}=="$cam_right_path", SYMLINK+="$CAM_RIGHT_SYMLINK"
EOF

if [[ -n "$cam_center_path" ]]; then
  cat >> "$RULE_FILE" <<EOF
SUBSYSTEM=="video4linux", KERNEL=="video*", ATTRS{idVendor}=="$CAM_VID", ATTRS{idProduct}=="$CAM_PID", ATTR{index}=="0", ENV{ID_PATH}=="$cam_center_path", SYMLINK+="$CAM_CENTER_SYMLINK"
EOF
fi

cat >> "$RULE_FILE" <<EOF

# ESP32C6 USB JTAG/serial: $ESP_VID:$ESP_PID
# Use serial to distinguish; also ignore ModemManager
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="$ESP_VID", ENV{ID_MODEL_ID}=="$ESP_PID", ENV{ID_SERIAL_SHORT}=="$esp_left_serial",  SYMLINK+="$ESP_LEFT_SYMLINK",  ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="$ESP_VID", ENV{ID_MODEL_ID}=="$ESP_PID", ENV{ID_SERIAL_SHORT}=="$esp_right_serial", SYMLINK+="$ESP_RIGHT_SYMLINK", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
EOF

chmod 644 "$RULE_FILE"
echo "Wrote: $RULE_FILE"

echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger
sleep 0.2

echo "Result:"
ls -l /dev/yubi_* || true
echo "Done."
