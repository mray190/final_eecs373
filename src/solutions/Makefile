SUBDIRS = task1 task2 
all:
	@for dir in $(SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

clean:
	@rm -f *~ bin/*
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]"; $(MAKE) -C $$dir clean || exit 2; done
