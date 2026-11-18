#!/bin/sh
# скрипт при старте контейнера

set -eu # завершать скрипт при ошибке вып. команды; считать обращение к неопр. переменной ошибкой


if [ "$#" -ne 1 ]; then # если кол-во аргументов != 1, вывести сообщение о том, как пользоваться
    printf '%s\n' 'Usage: /entrypoint.sh ping|T2|T3|T4|T5' >&2
    exit 64
fi

case "$1" in # выбираем модели
    ping)
        # Не использовать здесь ROS: stdout belongs to the JSON protocol.
        cd /opt/participant
        exec python3 -m participant.ping
        ;;
    T2|T3|T4|T5) # Не реализовано еще
        printf 'Task %s is not implemented.\n' "$1" >&2
        exit 64
        ;;
    *) # чет не то
        printf 'Unknown mode: %s\n' "$1" >&2
        exit 64
        ;;
esac
