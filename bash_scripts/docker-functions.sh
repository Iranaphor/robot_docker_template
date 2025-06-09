DOCKER_COMPOSE_DIR='./docker/docker-compose.yml'

function d_create () {
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   up --build -d ros2_dev
}

function d_deps () {
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -lc 'sudo apt-get update && \
                             rosdep update && \
                             rosdep install --from-paths src --ignore-src -r -y'
}

function d_up () {
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   up -d
}


function d_build_base () {
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -lc 'cd $HOME/base_ws && \
                             source /opt/ros/humble/setup.bash && \
                             colcon build --symlink-install'
}

function d_build_sensors () {
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -lc 'cd $HOME/sensors_ws && \
                             source /opt/ros/humble/setup.bash && \
                             colcon build --symlink-install'
}

function d_build_task () {
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -lc 'cd $HOME/task_ws && \
                             source /opt/ros/humble/setup.bash && \
                             colcon build --symlink-install'
}

function d_build_all () {
    d_build_base ;
    d_build_sensors ;
    d_build_task ;
}

function d_attach () {
    xhost +local:docker ;
    docker compose -f $DOCKER_COMPOSE_DIR \
                   --project-name robot_ros2 \
                   exec ros2_dev \
                   bash -l
}


function d_all () {
    d_create ; d_deps ; d_build_all ; d_attach
}

function d_start () {
    d_up ; d_build_task; d_attach
}
