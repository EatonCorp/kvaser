.PHONY: all
all:
	$(MAKE) -C canlib
	$(MAKE) -C leaf

.PHONY: install
install: all
	$(MAKE) -C canlib install
	$(MAKE) -C leaf install

.PHONY: clean
clean:
	$(MAKE) -C canlib clean
	$(MAKE) -C leaf clean

