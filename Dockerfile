# имя базового образа
ARG BASE_IMAGE=ros:humble-ros-base-jammy

# Собираем те же типы сообщений, что и в публичном симуляторе.
FROM ${BASE_IMAGE} AS t2-build
USER root
SHELL ["/bin/bash", "-e", "-o", "pipefail", "-c"]
ARG UNITREE_ROS2_REF=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
ARG GO1_SIM_REF=3473f086fd8a111592b4b9d822500c79f20c811a
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential ca-certificates curl python3-colcon-common-extensions \
       ros-humble-geometry-msgs ros-humble-rosidl-default-generators \
       ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/unitree_ros2
RUN curl -fsSL --retry 3 "https://codeload.github.com/unitreerobotics/unitree_ros2/tar.gz/${UNITREE_ROS2_REF}" \
       | tar xz --strip-components=1 \
    && source /opt/ros/humble/setup.bash \
    && colcon build --base-paths cyclonedds_ws/src/unitree/unitree_go \
       --merge-install --install-base /opt/go1_interfaces \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && mkdir -p /opt/go1_interfaces/licenses \
    && cp LICENSE /opt/go1_interfaces/licenses/unitree_ros2-LICENSE

# модель и список проверяемых пар должны быть из одной ревизии.
RUN root_url="https://raw.githubusercontent.com/Urika000/go1-simulator-public/${GO1_SIM_REF}" \
    && mkdir -p /opt/go1_model/assets \
    && for file in go1.xml scene.xml t2_collision_pairs.csv LICENSE \
           assets/calf.stl assets/hip.stl assets/thigh.stl assets/thigh_mirror.stl assets/trunk.stl; do \
        curl -fsSL --retry 3 "${root_url}/models/mujoco/unitree_go1/${file}" -o "/opt/go1_model/${file}"; \
    done \
    && curl -fsSL --retry 3 "${root_url}/LICENSE" -o /opt/go1_model/Project-MIT-LICENSE

# используем образ как основу
FROM ${BASE_IMAGE}

# root is used only while assembling the image.
# ассемблинг нужного
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       libgl1 libglfw3 python3-numpy python3-pip python3-zipp \
       ros-humble-geometry-msgs ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY config/t2-python.lock /tmp/t2-python.lock
RUN python3 -m pip install --no-cache-dir --no-deps --require-hashes -r /tmp/t2-python.lock \
    && rm /tmp/t2-python.lock
COPY --from=t2-build /opt/go1_interfaces /opt/go1_interfaces
COPY --from=t2-build /opt/go1_model /opt/go1_model

# создадим группу для запуска без рут прав
RUN groupadd --gid 10001 participant \
    && useradd --uid 10001 --gid 10001 --create-home --shell /bin/bash participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    HOME=/home/participant \
    SOLUTION_CONFIG=/opt/participant/solution.json

# рабочая директория и копирование нужных файлов
WORKDIR /opt/participant
COPY participant/ ./participant/
COPY solution.json ./solution.json
COPY entrypoint.sh /entrypoint.sh

# подготавливаем права для пользователей: разрешаем запуск, читать файлы и заходить в каталоги проекта
RUN sed -i 's/\r$//' /entrypoint.sh \
    && chmod 755 /entrypoint.sh \
    && chmod -R a+rX /opt/participant

USER participant:participant
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
