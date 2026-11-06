#!/bin/bash
# Видео из Gazebo: робот едет к цели через боевой Nav2, кадры рендерит сам
# симулятор (камеры-сенсоры ogre2, EGL на /dev/dri). На каждый мир — mp4 по
# каждой камере, общий mp4 «сверху | сзади-сбоку» и gif до SIM_VIDEO_GIF_MB.
#
# Камеры и метки целей ставит rob_box_sim.video_overlay в копию мира,
# цель — scenarios/video.yaml, едет стенд nav_bench (compose.video.yaml).
# Миры из карты квартиры (local/...) не снимаются: приватно.
#
# Из корня репо, образ уже собран (docker build -f docker/sim/Dockerfile ...):
#   SIM_IMAGE=rob_box_sim:local SIM_DRI_CARD=/dev/dri/card1 \
#     docker/sim/record_video.sh polygon turn90
#
# Переменные (кроме SIM_IMAGE/SIM_PREFIX/SIM_DRI_* из compose.yaml):
#   SIM_PROJECT       имя compose-проекта             (rob_box_sim_video)
#   SIM_VIDEO_OUT     куда класть видео               (docker/sim/video)
#   SIM_VIDEO_RATE    кадров в секунду sim time       (10)
#   SIM_VIDEO_WIDTH   ширина кадра камеры             (1280)
#   SIM_VIDEO_SPEED   ускорение видео против sim time (2)
#   SIM_VIDEO_LEAD_S  сколько секунд до старта цели   (2)
#   SIM_VIDEO_GIF_MB  предел размера gif, МБ          (10)
#   SIM_UP_TIMEOUT    сколько ждать nav2, с           (240)
set -euo pipefail

if [ "$#" -eq 0 ]; then
    echo "usage: $0 <мир> [мир ...]   (polygon turn90 corridor low_obstacle)" >&2
    exit 2
fi

here="$(cd "$(dirname "$0")" && pwd)"
project="${SIM_PROJECT:-rob_box_sim_video}"
prefix="${SIM_PREFIX:-rob-box-sim-video}"
out_root="${SIM_VIDEO_OUT:-$here/video}"
rate="${SIM_VIDEO_RATE:-10}"
speed="${SIM_VIDEO_SPEED:-2}"
lead_s="${SIM_VIDEO_LEAD_S:-2}"
gif_mb="${SIM_VIDEO_GIF_MB:-10}"
export SIM_PREFIX="$prefix" SIM_VIDEO_RATE="$rate"
export SIM_UID="${SIM_UID:-$(id -u)}"
export SIM_GID="${SIM_GID:-$(id -g)}"

dc() { docker compose -p "$project" -f "$here/compose.yaml" -f "$here/compose.video.yaml" "$@"; }
# ffmpeg и find — внутри контейнера gazebo: кадры пишет root, ffmpeg в образе сима.
in_gazebo() { docker exec "$prefix-gazebo" bash -c "$1"; }

# Сенсор пишет <полное имя сенсора>_<N>.png, N с нуля; полное имя в Fortress —
# «модель::звено::сенсор» (video_top::link::video_top_0.png).
# Номер последнего кадра камеры:
last_frame() {
    in_gazebo "ls /sim_video/frames/$1 2>/dev/null | sed -n 's/^.*_\([0-9]*\)\.png$/\1/p' | sort -n | tail -1"
}

# Кадры камеры start..end (включительно) → mp4 со скоростью ×speed.
encode() {
    local cam="$1" start="$2" end="$3" dst="$4" name
    name="$(in_gazebo "ls /sim_video/frames/$cam | head -1 | sed 's/_[0-9]*\.png$//'")"
    in_gazebo "ffmpeg -y -loglevel error -framerate $((rate * speed)) -start_number $start \
        -i 'file:/sim_video/frames/$cam/${name}_%d.png' -frames:v $((end - start + 1)) \
        -c:v libx264 -preset medium -crf 23 -pix_fmt yuv420p -movflags +faststart $dst"
}

# Сенсор Gazebo рендерит, только пока на его топик кто-то подписан (без
# подписчика кадров нет вовсе): подписчик — ros_gz_bridge в контейнере gazebo.
# Кадры после финиша камеры пишут, пока жив контейнер: encode берёт ровно start..end,
# а каталог кадров чистит cleanup уже после down.
start_subscriber() {
    docker exec -d "$prefix-gazebo" /entrypoint.sh ros2 run ros_gz_bridge parameter_bridge \
        '/sim/video/video_top@sensor_msgs/msg/Image[ignition.msgs.Image' \
        '/sim/video/video_chase@sensor_msgs/msg/Image[ignition.msgs.Image'
}

# Кадры и видео пишет root контейнера: удалить кадры и отдать файлы владельцу хоста.
cleanup() {
    docker run --rm --name "$prefix-cleanup" --entrypoint bash -v "$1:/sim_video" \
        "${SIM_IMAGE:-rob_box_sim:local}" -c "rm -rf /sim_video/frames && chown -R $SIM_UID:$SIM_GID /sim_video"
}

rc=0
for world in "$@"; do
    case "$world" in
        local/* | local)
            echo "❌ $world: карту квартиры не снимаем" >&2
            rc=1
            continue
            ;;
    esac
    out="$out_root/$world"
    mkdir -p "$out"
    export SIM_WORLD="$world" SIM_VIDEO_DIR="$out" SIM_RESULTS_DIR="$out/results"
    mkdir -p "$SIM_RESULTS_DIR"
    echo "=== видео $world ($(date -u +%FT%TZ)) → $out ==="
    dc up -d --no-build
    ok=0
    for _ in $(seq 1 "$(( ${SIM_UP_TIMEOUT:-240} / 5 ))"); do
        if [ "$(docker inspect -f '{{.State.Health.Status}}' "$prefix-nav2" 2>/dev/null)" = healthy ]; then
            ok=1
            break
        fi
        sleep 5
    done
    if [ "$ok" -ne 1 ]; then
        echo "❌ $world: nav2 не стал healthy за ${SIM_UP_TIMEOUT:-240} с"
        dc ps -a
        dc --profile bench down -v
        rc=1
        continue
    fi

    # Камеры включаются только теперь: пока поднимаются Nav2 и rtabmap, рендер не нужен.
    start_subscriber
    # Стенд в фоне: момент отправки цели нужен, чтобы отрезать простой до неё.
    bench_log="$out/bench.log"
    dc --profile bench run --rm --no-deps bench > "$bench_log" 2>&1 &
    bench_pid=$!
    sent=0 start_top="" start_chase=""
    while kill -0 "$bench_pid" 2>/dev/null; do
        if [ "$sent" -eq 0 ] && grep -q 'цель (' "$bench_log"; then
            sent=1
            start_top="$(last_frame video_top)"
            start_chase="$(last_frame video_chase)"
            echo "[video] цель отправлена: кадр video_top=${start_top:-нет} video_chase=${start_chase:-нет}"
        fi
        sleep 1
    done
    bench_rc=0
    wait "$bench_pid" || bench_rc=$?
    grep '^\[bench\] {' "$bench_log" || true
    [ "$bench_rc" -eq 0 ] || { echo "❌ $world: стенд упал ($bench_log)"; rc=1; }

    # Хвост lead_s после финиша.
    sleep "$lead_s"
    end_top="$(last_frame video_top)"
    end_chase="$(last_frame video_chase)"
    if [ -z "$end_top" ] || [ -z "$end_chase" ]; then
        echo "❌ $world: камеры не записали кадров (рендер? см. $out/gazebo.log)"
        dc logs --no-color --tail 300 gazebo > "$out/gazebo.log" 2>&1 || true
        dc --profile bench down -v
        cleanup "$out"
        rc=1
        continue
    fi
    lead=$((lead_s * rate))
    s_top=$(( ${start_top:-0} > lead ? ${start_top:-0} - lead : 0 ))
    s_chase=$(( ${start_chase:-0} > lead ? ${start_chase:-0} - lead : 0 ))
    echo "[video] кадры video_top $s_top..$end_top, video_chase $s_chase..$end_chase;" \
        "видео ×$speed ($((rate * speed)) fps)"
    encode video_top "$s_top" "$end_top" /sim_video/top.mp4
    encode video_chase "$s_chase" "$end_chase" /sim_video/chase.mp4
    # Общий кадр: сверху | сзади-сбоку, по 960×540.
    in_gazebo "ffmpeg -y -loglevel error -i /sim_video/top.mp4 -i /sim_video/chase.mp4 \
        -filter_complex '[0:v]scale=960:540[a];[1:v]scale=960:540[b];[a][b]hstack=inputs=2:shortest=1' \
        -c:v libx264 -preset medium -crf 24 -pix_fmt yuv420p -movflags +faststart /sim_video/$(basename "$world").mp4"
    # gif: ширина и fps вниз, пока не влезет в предел Telegram.
    for w in 960 800 640 480; do
        for fps in 10 8 6; do
            in_gazebo "ffmpeg -y -loglevel error -i /sim_video/$(basename "$world").mp4 \
                -vf 'fps=$fps,scale=$w:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle' \
                /sim_video/$(basename "$world").gif"
            size=$(stat -c %s "$out/$(basename "$world").gif")
            echo "[video] gif ${w}px ${fps} fps: $((size / 1024)) КБ"
            [ "$size" -le $((gif_mb * 1024 * 1024)) ] && break 2
        done
    done
    dc logs --no-color --tail 400 gazebo nav2 > "$out/stack.log" 2>&1 || true
    dc --profile bench down -v
    cleanup "$out"
    ls -l "$out"
done
exit "$rc"
