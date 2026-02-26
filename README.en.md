# edge-motion (English)

`edge-motion` is a small Linux daemon that provides **edge scrolling**: hold your finger near a touchpad edge to generate smooth scrolling or cursor motion.

## Quick start (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libevdev-dev libudev-dev zenity
make build
sudo make install
sudo make install-service
```

## Helpful commands

```bash
sudo edge-motion-config
sudo /usr/local/bin/edge-motion --list-devices
```

Configuration file:

- `/etc/default/edge-motion` (`EDGE_MOTION_ARGS`)

Auto-update config file:

- `/etc/default/edge-motion-update`

For detailed usage examples and tuning profiles, see the Russian guide in `README.ru.md` (until a full English manual is added).
