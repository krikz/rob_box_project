# Пакеты Zenoh для ROS 2 Humble

В репозитории закреплены сборки серии `0.1.11-1jammy` для ARM64 и AMD64.
Для каждой архитектуры используются два файла `.deb` из официального репозитория ROS APT.

- `ros-humble-rmw-zenoh-cpp_0.1.11-1jammy.20261106.124730_arm64.deb`
- `ros-humble-zenoh-cpp-vendor_0.1.11-1jammy.20261027.195405_arm64.deb`
- `ros-humble-rmw-zenoh-cpp_0.1.11-1jammy.20261105.191826_amd64.deb`
- `ros-humble-zenoh-cpp-vendor_0.1.11-1jammy.20261031.173316_amd64.deb`

`../scripts/download_rmw_debs.sh <каталог> [arm64|amd64]` скачивает пакеты
через APT с настроенным индексом ROS Humble для Jammy выбранной архитектуры.
Сборки с `RMW_ZENOH_VENDOR=false` устанавливают пакеты той же серии через ROS APT
пока серия доступна в индексе.
