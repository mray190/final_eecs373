include ../../common.mk
# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
	 $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES)


CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) 

LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	$(LDFLAGS_IMAGESOURCE) \
	$(LDFLAGS_MATH) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCMTYPES) \
	$(LDFLAGS_EECS467) \
	$(LDFLAGS_MAPPING)

LIBDEPS = $(call libdeps, $(LDFLAGS))
LCMTYPES = ../lcmtypes

BIN_SOLUTIONS_TASK1 = $(BIN_PATH)/task1
BIN_SOLUTIONS_TASK12 = $(BIN_PATH)/task12
BIN_SOLUTIONS_TURN_PRODUCER = $(BIN_PATH)/turnProducer

ALL = $(LIB_EECS467) $(BIN_SOLUTIONS_TASK1) $(BIN_SOLUTIONS_TASK12) $(BIN_SOLUTIONS_TURN_PRODUCER)

all: $(ALL)

$(BIN_SOLUTIONS_TASK1):task1.o rexarmController.o Handler.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_SOLUTIONS_TASK12):task12.o rexarmController2.o Handler.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_SOLUTIONS_TURN_PRODUCER):turnProducer.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)

%.o: %.cpp %.hpp
	@echo "    $@"
	@$(CXX) $(CXXFLAGS) -c $<
