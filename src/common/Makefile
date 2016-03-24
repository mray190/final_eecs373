include ../common.mk

CFLAGS  = $(CFLAGS_STD)  $(CFLAGS_GLIB)  $(CFLAGS_GTK) -O4 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_GTK)
LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_COMMON = $(LIB_PATH)/libcommon.a
LIBCOMMON_OBJS = \
	c5.o \
	config.o \
	encode_bytes.o \
	getopt.o \
	homography.o \
	io_util.o \
	ioutils.o \
	matd.o \
       param_widget.o \
	pg.o \
	serial.o \
	serial_util.o \
	ssocket.o \
	string_util.o \
	svd22.o \
	task_thread.o \
	timespec.o \
	timestamp.o \
	time_util.o \
	timesync.o \
	unionfind.o \
	url_parser.o \
	varray.o \
	vhash.o \
	workerpool.o \
	zarray.o \
	zhash.o \
	zmaxheap.o

all: $(LIB_COMMON)

$(LIB_COMMON): $(LIBCOMMON_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(LIB_COMMON)
