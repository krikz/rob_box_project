# Makefile для тестовой Docker-compose среды voice assistant

IMAGE_PREFIX ?= ghcr.io/krikz/rob_box
IMAGE_TAG    ?= latest
ROS_DISTRO   ?= kilted
PLATFORM     ?= linux/amd64

COMPOSE = docker compose -f docker-compose.test.yml
ENV     = SERVICE_IMAGE_PREFIX=$(IMAGE_PREFIX) IMAGE_TAG=$(IMAGE_TAG) ROS_DISTRO=$(ROS_DISTRO) PLATFORM=$(PLATFORM)

.PHONY: test test-arm build logs clean results

## Запустить все сценарии (amd64, быстро)
test:
	@echo ">>> Running integration tests ($(PLATFORM))"
	@mkdir -p results
	$(ENV) $(COMPOSE) up \
		--abort-on-container-exit \
		--exit-code-from scenario-runner \
		--build
	@$(MAKE) results

## Запустить с QEMU на ARM образе (тестирует реальный prod image)
test-arm:
	@echo ">>> Enabling QEMU for arm64..."
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	@$(MAKE) test PLATFORM=linux/arm64

## Только собрать scenario-runner образ (ollama — официальный, не требует сборки)
build:
	$(ENV) $(COMPOSE) build scenario-runner

## Следить за логами во время прогона
logs:
	$(ENV) $(COMPOSE) logs -f

## Показать последние результаты
results:
	@if [ -f results/test_results.json ]; then \
		python3 -c " \
import json; \
d = json.load(open('results/test_results.json')); \
print(f\"Results: {d['passed']}/{d['total']} passed\"); \
[print(f\"  {'OK' if s['passed'] else 'FAIL'} {s['name']}\") for s in d['scenarios']] \
		"; \
	else \
		echo "No results yet. Run 'make test' first."; \
	fi

## Остановить и удалить все тест-контейнеры
clean:
	$(ENV) $(COMPOSE) down --remove-orphans
	rm -f results/test_results.json
