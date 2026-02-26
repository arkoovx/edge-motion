PREFIX ?= /usr/local
BINDIR ?= $(PREFIX)/bin
UNITDIR ?= /etc/systemd/system

APP := edge-motion
SRC := edge-motion.c
CFLAGS ?= -O2
LIBS := $(shell pkg-config --libs libevdev libudev 2>/dev/null)
CPPFLAGS += $(shell pkg-config --cflags libevdev libudev 2>/dev/null)
LDFLAGS += -pthread

.PHONY: help build clean install uninstall install-service uninstall-service deps-check

help:
	@echo "Targets:"
	@echo "  make build              Build binary ($(APP))"
	@echo "  make install            Install binary to $(BINDIR)"
	@echo "  make uninstall          Remove binary from $(BINDIR)"
	@echo "  make install-service    Install systemd unit and enable service"
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

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(APP)

install-service: install
	install -m 0644 systemd/edge-motion.service $(DESTDIR)$(UNITDIR)/edge-motion.service
	systemctl daemon-reload
	systemctl enable --now edge-motion.service

uninstall-service:
	-systemctl disable --now edge-motion.service
	rm -f $(DESTDIR)$(UNITDIR)/edge-motion.service
	systemctl daemon-reload
