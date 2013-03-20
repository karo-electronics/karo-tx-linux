TARGETS = breakpoints
TARGETS += kcmp
TARGETS += mqueue
TARGETS += vm
TARGETS += cpu-hotplug
TARGETS += memory-hotplug
TARGETS += efivarfs
TARGETS += net-socket
TARGETS += net-afpacket
TARGETS += timers

all:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET; \
	done;

run_tests: all
	for TARGET in $(TARGETS); do \
		make -C $$TARGET run_tests; \
	done;

clean:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET clean; \
	done;
