#!/usr/bin/env bash
# Define container name
CONTAINER_NAME="operator"
SOURCE="git"

# Function to prompt for confirmation
confirm() {
    read -r -p "$1 [y/N] " response
    case "$response" in
        [yY][eE][sS]|[yY]) 
            true
            ;;
        *)
            false
            ;;
    esac
}

# Instructions for how to interact with container
containerinstructions() {
    echo "To open a bash shell in the container, run:"
    echo "docker exec -it $CONTAINER_NAME bash"
}

# Check if ROS_DOMAIN_ID and HUSARNET_JOIN_CODE flags are provided
if [[ $# -lt 5 ]]; then
    echo "Usage: $0 -j <HUSARNET_JOIN_CODE> -d <ROS_DOMAIN_ID> -t <WALL_PANELS_TEAM_NAME> [-s <SOURCE>] -n [OPERATOR_NAME]"
    exit 1
fi

# Parse command-line arguments
while getopts ":j:d:t:n:s:" opt; do
    case ${opt} in
        j )
            COMPOSE_HUSARNET_JOIN_CODE=$OPTARG
            ;;
        d )
            COMPOSE_ROS_DOMAIN_ID=$OPTARG
            ;;
        t )
            if [[ $OPTARG =~ ^wall-panels-team-[0-9]+_[0-9]+$ ]]; then
                COMPOSE_WALL_PANELS_TEAM_NAME=$OPTARG
            else
                echo "Invalid -t argument. Format must be wall-panels-team-NUMBER_NUMBER" 1>&2
                exit 1
            fi
            ;;
        n )
            OPERATOR_NAME=$OPTARG
            ;;
        s )
            SOURCE=$OPTARG
            ;;
        \? )
            echo "Invalid option: $OPTARG" 1>&2
            exit 1
            ;;
        : )
            echo "Invalid option: $OPTARG requires an argument" 1>&2
            exit 1
            ;;
    esac
done
shift $((OPTIND -1))

# Check if there are any unexpected arguments or if the -n flag is used incorrectly
if [ "$#" -gt 0 ] || { [ -z "$OPERATOR_NAME" ] && [ "$OPTIND" -eq 2 ]; }; then
    usage
fi

# If OPERATOR_NAME is not provided, generate a random one
if [ -z "$OPERATOR_NAME" ]; then
    OPERATOR_NAME=$(tr -dc 'a-zA-Z0-9' < /dev/urandom | fold -w 16 | head -n 1)
fi

echo "Operator Name: $OPERATOR_NAME"

echo "COMPOSE_HUSARNET_JOIN_CODE=$COMPOSE_HUSARNET_JOIN_CODE" > .env
{
echo "COMPOSE_HUSARNET_HOSTNAME=operator-$OPERATOR_NAME"
echo "COMPOSE_ROS_DOMAIN_ID=$COMPOSE_ROS_DOMAIN_ID"
echo "COMPOSE_WALL_PANELS_TEAM_NAME=$COMPOSE_WALL_PANELS_TEAM_NAME"
} >> .env

# configure host network to work with husranet
sudo sysctl -w net.ipv6.conf.all.disable_ipv6=0
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=0
sudo ufw allow in on hnet0 from fc94::/16

# Check if the source option is provided
if [[ -z "$SOURCE" ]]; then
    echo "Source option (-s) is required. Specify 'git' or 'docker'."
    exit 1
fi

# Check if the source option is valid
if [[ "$SOURCE" != "git" && "$SOURCE" != "docker" ]]; then
    echo "Invalid source option. Specify 'git' or 'docker'."
    exit 1
fi

# Define repository URLs
REPO_URLS=(
    "git@bitbucket.org:botbuilt/bb_msgs.git"
    "git@bitbucket.org:botbuilt/bb_transforms.git"
    "git@bitbucket.org:botbuilt/utils.git"
    "git@bitbucket.org:botbuilt/robot_operation.git"
    #"git@bitbucket.org:botbuilt/panel_planner.git"
    #"git@bitbucket.org:botbuilt/panel_scheduler.git"
)

# Clone or pull the source code
if [[ "$SOURCE" == "git" ]]; then
    mkdir -p bb_ws/src
    cd bb_ws/src || exit
    for REPO_URL in "${REPO_URLS[@]}"; do
        REPO_NAME=$(basename -s .git "$REPO_URL")
        if [[ ! -d "$REPO_NAME" ]]; then
            git clone "$REPO_URL"
        else
            cd "$REPO_NAME" || exit
            git pull
            cd ..
        fi
    done
    cd ../..
else
    # Example docker pull, adjust as needed
    docker pull "botbuilt/amd64-bb-msgs"
fi

# Check if the containers managed by the specified docker compose file exist
if sudo docker ps -a --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
    confirm "Containers defined in the $CONTAINER_NAME compose.yaml file will be destroyed. Do you want to proceed?" && \
    sudo -E docker compose -f ./compose.yaml down
    xhost -local:
fi

# Build and start the container
sudo -E docker compose -f ./compose.yaml up -d --build && \
# Execute colcon build inside the container
sudo docker exec -it $CONTAINER_NAME bash -c "cd /bb_ws && source /opt/ros/humble/setup.bash && colcon build"
sudo docker exec -it $CONTAINER_NAME bash -c "cd /bb_ws && chmod +x /bb_ws/install/setup.bash && source /bb_ws/install/setup.bash"

# allow any connections that originate locally (needed for running firefox from docker)
xhost +local:

containerinstructions
