# edge-motion

`edge-motion` — это маленький Linux-демон, который добавляет **edge scrolling**: удерживаете палец у края тачпада → получаете плавный скролл/движение.

> Коротко: удобно, если штатная прокрутка тачпада неудобная, нестабильная или плохо настраивается.

---

## Что вы получите

- Автовыбор подходящего тачпада.
- Режим **motion** (по умолчанию) и режим **scroll**.
- Тонкую настройку чувствительности/ускорения.
- Восстановление после переподключения устройства.
- Запуск как `systemd`-сервис (автостарт после ребута).
- Лёгкий UI-конфигуратор: графический интерфейс (zenity) с кнопками/ползунками и fallback в TUI.
- Безопасный автоапдейт перед стартом сервиса (если указан репозиторий исходников).

---

## Быстрый старт (Ubuntu/Debian)

### 1) Зависимости

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libevdev-dev libudev-dev zenity
```

### 2) Сборка + установка

```bash
make build
sudo make install
sudo make install-service
```

### 3) Проверка, что сервис жив

```bash
systemctl status edge-motion.service
journalctl -u edge-motion.service -f
```

---

## 2 самых полезных команды

```bash
sudo edge-motion-config
sudo /usr/local/bin/edge-motion --list-devices
```

- Первая открывает понятное меню и сохраняет настройки в `/etc/default/edge-motion`.
- Вторая помогает, если нужно явно выбрать устройство (`--device /dev/input/eventX`).
- Для ручного запуска автообновления без перезагрузки: `sudo /usr/local/bin/edge-motion-auto-update` (или `sudo make update-now`).

---

## Где лежат настройки

### Основной runtime-конфиг

Файл: `/etc/default/edge-motion`

Используется переменная:

```bash
EDGE_MOTION_ARGS="..."
```

### Конфиг автообновления

Файл: `/etc/default/edge-motion-update`

```bash
EDGE_MOTION_AUTO_UPDATE=1
EDGE_MOTION_REPO_DIR=/opt/edge-motion-src
EDGE_MOTION_UPDATE_BRANCH=main
```

Если репозиторий не найден — автообновление просто пропускается, установленная рабочая версия не ломается.

---

## Рекомендуемые стартовые профили

### Профиль A: «мягкий» (обычно самый комфортный)

```bash
EDGE_MOTION_ARGS="--no-grab --mode motion --threshold 0.06 --hold-ms 90 --pulse-ms 12 --pulse-step 1.5 --max-speed 2.4 --accel-exponent 1.6 --deadzone 0.07 --scroll-axis-priority dominant"
```

### Профиль B: «быстрый»

```bash
EDGE_MOTION_ARGS="--no-grab --mode scroll --threshold 0.05 --hold-ms 70 --pulse-ms 10 --pulse-step 1.8 --max-speed 2.8 --accel-exponent 1.8 --deadzone 0.06 --scroll-axis-priority dominant"
```

После правок примените:

```bash
sudo systemctl daemon-reload
sudo systemctl restart edge-motion.service
```

---

## Понятно про ключевые опции

- `--mode scroll|motion` — скролл колёсиком или движение курсора.
- `--threshold 0.06` — ширина «краевой» зоны (меньше = легче срабатывает).
- `--hold-ms 80` — задержка до старта (больше = меньше случайных срабатываний).
- `--pulse-ms 10` — частота импульсов.
- `--pulse-step 1.5` — базовый шаг.
- `--max-speed 3.0` — ограничение максимального ускорения.
- `--accel-exponent 1.0+` — нелинейный разгон ближе к краю.
- `--deadzone 0.0..0.49` — центральная зона без активации.
- `--natural-scroll` — natural-направление вертикального скролла.
- `--diagonal-scroll` — разрешить диагональный скролл.
- `--two-finger-scroll` — edge-scroll только при 2 пальцах (в scroll-режиме).
- `--grab/--no-grab` — эксклюзивный/совместный захват тачпада.

---

## Если что-то не работает

### 1) Устройство не найдено

```bash
sudo /usr/local/bin/edge-motion --list-devices
```

Дальше укажите устройство явно:

```bash
EDGE_MOTION_ARGS="--device /dev/input/eventX --no-grab --mode scroll --threshold 0.06 --hold-ms 90 --pulse-ms 12 --pulse-step 1.5"
```

### 2) Срабатывает слишком резко

- Увеличьте `--hold-ms` (например, 90 → 120).
- Уменьшите `--pulse-step` (например, 1.5 → 1.2).
- Увеличьте `--threshold` (например, 0.06 → 0.08).

### 3) Перестал работать обычный тачпад

Проверьте, что не включён `--grab`.

### 4) Ошибка про `uinput`

Нужны root-права (через systemd сервис обычно всё корректно).

---

## Диагностика

```bash
sudo /usr/local/bin/edge-motion --verbose
/usr/local/bin/edge-motion --help
/usr/local/bin/edge-motion --version
```

---

## Удаление

```bash
sudo make uninstall-service
sudo make uninstall
```

---

## Лицензия

Пока нет отдельного файла `LICENSE`. Если планируете использование в продукте/дистрибутиве — сначала добавьте лицензию.
