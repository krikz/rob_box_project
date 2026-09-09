# Скачанные .deb для rmw_zenoh_cpp 0.1.8 (вендорятся в репозиторий).
# Для полной reproducible-сборки:
#   1) docker/base/vendor/*.deb — коммитятся
#   2) docker/base/scripts/download_rmw_debs.sh — пере-скачивает из
#      snapshots.ros.org если нужно поднять timestamp SHA (например, для
#      пере-сборки под другую архитектуру)
#
# Чтобы удалить vendor (и уйти в network fallback в Dockerfile):
#   rm docker/base/vendor/*.deb
# Docker build с RMW_ZENOH_VENDOR=true упадёт на COPY vendor/. — это
# преднамеренный fail-fast (см. ADR-0088 / issue #2281).
*.deb.asc
*.deb.sha256sum
