## edge-motion

Утилита для Linux, которая двигает курсор, если палец удерживается у края тачпада (аналог edge scrolling).

## Что уже исправлено в коде

- Исправлен анти-дребезг: таймер `hold_ms` обновляется только по завершенному кадру (`SYN_REPORT`) и только при реальном смещении выше порога.
- Исправлен busy-loop при отключении устройства: обработка `POLLERR|POLLHUP|POLLNVAL` теперь уводит в reconnect.
- Усилена надежность запуска/остановки: проверки `ioctl`, аккуратный cleanup и safe join потока.

## Быстрый старт (Ubuntu)

### 1) Установить зависимости

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libevdev-dev libudev-dev
```

### 2) Собрать

```bash
make build
```

или напрямую:

```bash
gcc -O2 edge-motion.c -o edge-motion $(pkg-config --cflags --libs libevdev libudev) -pthread
```

### 3) Установить бинарник

```bash
sudo make install
```

## Systemd (автозапуск)

В репозитории уже есть готовый юнит: `systemd/edge-motion.service`.

### Вариант A (через Makefile)

```bash
sudo make install-service
```

### Вариант B (вручную)

```bash
sudo install -m 0644 systemd/edge-motion.service /etc/systemd/system/edge-motion.service
sudo systemctl daemon-reload
sudo systemctl enable --now edge-motion.service
```

Проверка:

```bash
systemctl status edge-motion.service
journalctl -u edge-motion.service -f
```

## Настройка

Отредактируйте `ExecStart` в `/etc/systemd/system/edge-motion.service`:

- `--threshold 0.06` — зона у края (0.04 меньше, 0.08 больше).
- `--hold-ms 80` — задержка перед стартом движения.
- `--pulse-ms 10` — интервал импульсов.
- `--pulse-step 3` — скорость движения.

После правок:

```bash
sudo systemctl daemon-reload
sudo systemctl restart edge-motion.service
```

## Удаление

```bash
sudo make uninstall-service
sudo make uninstall
```

или вручную:

```bash
sudo systemctl disable --now edge-motion.service
sudo rm -f /usr/local/bin/edge-motion
sudo rm -f /etc/systemd/system/edge-motion.service
sudo systemctl daemon-reload
```

## Если не собирается в контейнере/CI

Проверьте, видит ли `pkg-config` нужные библиотеки:

```bash
pkg-config --modversion libevdev libudev
```

Если `Package ... not found` — не хватает dev-пакетов.

Если `apt update` в CI/sandbox дает `403 Forbidden`, это ограничение окружения, а не ошибка кода. В таком случае собирайте локально на Ubuntu/сервере с рабочими APT-репозиториями.
