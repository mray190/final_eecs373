include ../common.mk

CFLAGS =   $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_GTK)  $(CFLAGS_USB)  $(CFLAGS_PNG)  $(CFLAGS_DC1394)
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_GTK) $(LDFLAGS_USB) $(LDFLAGS_PNG) $(LDFLAGS_DC1394)
LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_APRIL = $(LIB_PATH)/libapril.a
LIBAPRIL_OBJS = \
	apriltag.o \
	apriltag_quad_thresh.o \
	tag16h5.o \
	tag25h7.o \
	tag25h9.o \
	tag36h10.o \
	tag36h11.o \
	tag36artoolkit.o \
	g2d.o

BIN_DEMO= $(BIN_PATH)/apriltag_demo

ALL = $(LIB_APRIL) $(BIN_DEMO)

all: $(ALL)

$(LIB_APRIL): $(LIBAPRIL_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_DEMO): apriltag_demo.o $(LIB_APRIL) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_STD)

clean:
	@rm -f *~ *.o
	@rm -f $(ALL)
