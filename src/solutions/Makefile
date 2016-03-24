include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_APRIL) \
	$(CFLAGS_LCMTYPES) \
	-O3

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_APRIL) \
	  $(LDFLAGS_EECS467) \
	  $(LDFLAGS_MAPPING)

LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_SOLUTIONS_TASK2 = $(BIN_PATH)/runner

ALL = $(LIB_EECS467) $(BIN_SOLUTIONS_TASK2)

all: $(ALL)


$(BIN_SOLUTIONS_TASK2): Gui.o main.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -g3 -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)

%.o: %.cpp %.hpp
	@echo "    $@"
	@$(CXX) $(CXXFLAGS) -c $<
