DOCKER_COMPOSE_DIR='./docker/docker-compose.yml'

########################################################

function d_create () {
    cd $HOME/docker_workspaces/robot_docker_ros2 ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   up --build -d ros2_dev
}

########################################################

function d_deps () {
    cd $HOME/docker_workspaces/robot_docker_ros2 ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -lc 'cd $HOME/$1 && \
                             sudo apt-get update && \
                             rosdep update && \
                             rosdep install --from-paths src --ignore-src -r -y'
}
function d_deps_base () { d_deps base_ws ; }
function d_deps_sensors () { d_deps sensors_ws ; }
function d_deps_autonomy () { d_deps autonomy_ws ; }
function d_deps_control () { d_deps control_ws ; }
function d_deps_environment () { d_deps environment_ws ; }
function d_deps_all () {
    d_deps_base ;
    d_deps_sensors ;
    d_deps_autonomy ;
    d_deps_control ;
    d_deps_environment ;
}

########################################################

function d_build () {
    cd $HOME/docker_workspaces/robot_docker_ros2 ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -lc 'cd $HOME/$1 && \
                             source /opt/ros/humble/setup.bash && \
                             colcon build --symlink-install'
}
function d_build_base () { d_deps base_ws ; }
function d_build_sensors () { d_deps sensors_ws ; }
function d_build_autonomy () { d_deps autonomy_ws ; }
function d_build_control () { d_deps control_ws ; }
function d_build_environment () { d_deps environment_ws ; }
function d_build_all () {
    d_build_base ;
    d_build_sensors ;
    d_build_autonomy ;
    d_build_control ;
}

########################################################

function d_attach () {
    cd $HOME/docker_workspaces/robot_docker_ros2 ;
    ./bash_scripts/can_host_setup.sh ;
    xhost +local:docker ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -l
}

function d_up () {
    cd $HOME/docker_workspaces/robot_docker_ros2 ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   up -d
}

function d_down () {
    cd $HOME/docker_workspaces/robot_docker_ros2 ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   down
}

########################################################

function d_all () {
    d_create ; d_deps_all ; d_build_all ; d_attach
}

function d_start () {
    d_up ; d_attach
}

function d_relaunch () {
    d_down ; d_up ; d_attach
}

########################################################

