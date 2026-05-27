#!/bin/bash
set -e
trap : SIGTERM SIGINT

IMAGE="vins-fusion-ros2"
ROS_WS="/ros2_ws"

function echoUsage() {
    cat >&2 <<'EOF'
Usage:
  Live / rosbag mode (waits for ROS2 topics):
    ./run.sh [-l] [-g] <config_file>
        -l  also run loop_fusion node
        -g  also run global_fusion node

  Dataset mode (reads images from disk):
    ./run.sh --dataset --preset <name> --data <folder> <config_file>
    ./run.sh --dataset --left <folder> --right <folder> \
             [--times <file>] [--imu <csv>] [--ext <.png>] \
             [--time-scale <factor>] <config_file>

  Presets: kitti_odom (no IMU), euroc, tum_vi (IMU auto-detected)
EOF
}

if [ "$#" -lt 1 ]; then
    echoUsage
    exit 1
fi

DATASET=0
DATASET_ARGS=()
HOST_MOUNTS=()
LOOP_FUSION=0
GLOBAL_FUSION=0
POSITIONAL=()

# Convert a host path to its in-container path and record a read-only mount.
function add_mount_path() {
    local host_path
    host_path="$(realpath "$1")"
    if [ ! -e "$host_path" ]; then
        echo "Error: path not found: $host_path" >&2
        exit 1
    fi
    local container_path="/data${host_path}"
    HOST_MOUNTS+=("-v" "${host_path}:${container_path}:ro")
    echo "$container_path"
}

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) echoUsage; exit 0 ;;
        -l) LOOP_FUSION=1; shift ;;
        -g) GLOBAL_FUSION=1; shift ;;
        --dataset) DATASET=1; shift ;;
        --preset|--ext|--time-scale)
            DATASET_ARGS+=("$1" "$2"); shift 2 ;;
        --data|--left|--right|--times|--imu)
            mapped="$(add_mount_path "$2")"
            DATASET_ARGS+=("$1" "$mapped"); shift 2 ;;
        --) shift; while [ $# -gt 0 ]; do POSITIONAL+=("$1"); shift; done ;;
        -*) echo "Unknown flag: $1" >&2; echoUsage; exit 1 ;;
        *) POSITIONAL+=("$1"); shift ;;
    esac
done

if [ "${#POSITIONAL[@]}" -lt 1 ]; then
    echo "Error: missing config file" >&2
    echoUsage
    exit 1
fi

CONFIG_FILE="$(realpath "${POSITIONAL[-1]}")"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: config file not found: $CONFIG_FILE" >&2
    exit 1
fi

CONFIG_DIR="$(dirname "$CONFIG_FILE")"
CONFIG_BASENAME="$(basename "$CONFIG_FILE")"
CONTAINER_CONFIG="${ROS_WS}/config/${CONFIG_BASENAME}"

EXTRA_NODES=""
if [ $LOOP_FUSION -eq 1 ]; then
    EXTRA_NODES="ros2 run loop_fusion loop_fusion_node ${CONTAINER_CONFIG} &"
fi
if [ $GLOBAL_FUSION -eq 1 ]; then
    EXTRA_NODES="${EXTRA_NODES} ros2 run global_fusion global_fusion_node &"
fi

if [ $DATASET -eq 1 ]; then
    MAIN_CMD="ros2 run vins dataset_node ${CONTAINER_CONFIG} ${DATASET_ARGS[*]}"
else
    MAIN_CMD="ros2 run vins vins_node ${CONTAINER_CONFIG}"
fi

TTY_FLAG=$([ -t 0 ] && echo "-t" || echo "")

docker run \
    -i ${TTY_FLAG} \
    --rm \
    --net=host \
    -v "${CONFIG_DIR}:${ROS_WS}/config:ro" \
    "${HOST_MOUNTS[@]}" \
    "${IMAGE}" \
    bash -c "${EXTRA_NODES} ${MAIN_CMD}"
