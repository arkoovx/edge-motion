PREFIX ?= /usr/local
BINDIR ?= $(PREFIX)/bin
UNITDIR ?= /etc/systemd/system

APP := edge-motion
SRC := edge-motion.c
CFLAGS ?= -O2
LIBS := $(shell pkg-config --libs libevdev libudev 2>/dev/null)
CPPFLAGS += $(shell pkg-config --cflags libevdev libudev 2>/dev/null)
LDFLAGS += -pthread -lm

.PHONY: all help build clean install uninstall install-service uninstall-service deps-check install-config

all: build

help:
	@echo "Targets:"
	@echo "  make build              Build binary ($(APP))"
	@echo "  make install            Install binary and helper tools to $(BINDIR)"
	@echo "  make uninstall          Remove binary and helper tools from $(BINDIR)"
	@echo "  make install-service    Install systemd unit, default config and enable service"
	@echo "  make uninstall-service  Disable service and remove unit"
	@echo "  make clean              Remove build artifacts"
	@echo "  make deps-check         Check required dev packages via pkg-config"

deps-check:
	@pkg-config --exists libevdev libudev || \
		( echo "Missing deps: install libevdev-dev libudev-dev pkg-config" && exit 1 )
	@echo "Dependencies found: libevdev libudev"

build: deps-check
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SRC) -o $(APP) $(LIBS) $(LDFLAGS)

clean:
	rm -f $(APP)

install: build
	install -d $(DESTDIR)$(BINDIR)
	install -m 0755 $(APP) $(DESTDIR)$(BINDIR)/$(APP)
	install -m 0755 scripts/edge-motion-config $(DESTDIR)$(BINDIR)/edge-motion-config
	install -m 0755 scripts/edge-motion-auto-update $(DESTDIR)$(BINDIR)/edge-motion-auto-update

install-config:
	install -d $(DESTDIR)/etc/default
	@if [ ! -f $(DESTDIR)/etc/default/edge-motion ]; then \
		echo '# Runtime args for edge-motion service' > $(DESTDIR)/etc/default/edge-motion; \
		echo 'EDGE_MOTION_ARGS="--no-grab --mode scroll --threshold 0.06 --hold-ms 90 --pulse-ms 12 --pulse-step 1.5 --max-speed 2.4 --accel-exponent 1.6 --deadzone 0.07 --scroll-axis-priority dominant"' >> $(DESTDIR)/etc/default/edge-motion; \
	fi
	@if [ ! -f $(DESTDIR)/etc/default/edge-motion-update ]; then \
		echo '# Auto-update settings for edge-motion' > $(DESTDIR)/etc/default/edge-motion-update; \
		echo 'EDGE_MOTION_AUTO_UPDATE=1' >> $(DESTDIR)/etc/default/edge-motion-update; \
		echo 'EDGE_MOTION_REPO_DIR=/opt/edge-motion-src' >> $(DESTDIR)/etc/default/edge-motion-update; \
		echo 'EDGE_MOTION_UPDATE_BRANCH=main' >> $(DESTDIR)/etc/default/edge-motion-update; \
	fi

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(APP)
	rm -f $(DESTDIR)$(BINDIR)/edge-motion-config
	rm -f $(DESTDIR)$(BINDIR)/edge-motion-auto-update

install-service: install install-config
	install -m 0644 systemd/edge-motion.service $(DESTDIR)$(UNITDIR)/edge-motion.service
	@if command -v systemctl >/dev/null 2>&1; then \
		systemctl daemon-reload; \
		systemctl enable --now edge-motion.service; \
	else \
		echo "systemctl not found: service file installed only"; \
	fi

uninstall-service:
	@if command -v systemctl >/dev/null 2>&1; then \
		systemctl disable --now edge-motion.service || true; \
		systemctl daemon-reload; \
	fi
	rm -f $(DESTDIR)$(UNITDIR)/edge-motion.service
