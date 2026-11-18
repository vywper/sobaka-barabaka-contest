# имя базового образа
ARG BASE_IMAGE=ros:humble-ros-base-jammy

# используем образ как основу
FROM ${BASE_IMAGE}

# root is used only while assembling the image.
# ассемблинг нужного
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

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
