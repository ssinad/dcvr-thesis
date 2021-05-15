# CCFLAGS = -std=c++11 -g
UNAME := $(shell uname)

MAKEFLAGS += -r
ifeq ($(UNAME), Darwin)
	SYSTEM = x86-64_osx
else
	SYSTEM = x86-64_linux
endif
LIBFORMAT  = static_pic

ifeq ($(UNAME), Darwin)
	CPLEXDIR      = /Applications/CPLEX_Studio128/cplex
	CONCERTDIR    = /Applications/CPLEX_Studio128/concert
else
	# CPLEXDIR      = ~/ibm/ILOG/CPLEX_Studio128/cplex
	CPLEXDIR      = ./ibm/ILOG/CPLEX_Studio128/cplex
	CONCERTDIR    = ./ibm/ILOG/CPLEX_Studio128/concert
	# CONCERTDIR    = ~/ibm/ILOG/CPLEX_Studio128/concert
endif

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include
CCOPT = -m64 -O -std=c++11 -g -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG

# COPT  = -m64 -fPIC -fno-strict-aliasing
# JOPT  = -classpath $(CPLEXDIR)/lib/cplex.jar -O

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CPPSRC = src
TESTS = tests
CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
# CPLEXJARDIR   = $(CPLEXDIR)/lib/cplex.jar
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNDIRS  = -L $(CPLEXLIBDIR) -L $(CONCERTLIBDIR)
CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread -ldl
CCFLAGS = $(CCOPT) -I $(CPPSRC)
CCOBJ = $(CCFLAGS) -c

dcvr.o: $(CPPSRC)/dcvr.cpp
	# g++ -c $(CCFLAGS)  -o dcvr.o dcvr.cpp
	g++ $(CCOBJ) -DIL_STD -I $(CPLEXINCDIR) -I $(CONCERTINCDIR) -o dcvr.o $(CPPSRC)/dcvr.cpp
iterPCA.o: $(CPPSRC)/iterPCA.cpp
	g++ $(CCOBJ) -o iterPCA.o $(CPPSRC)/iterPCA.cpp
PcaReader.o: $(CPPSRC)/PcaReader.cpp
	g++ $(CCOBJ) -o PcaReader.o $(CPPSRC)/PcaReader.cpp
DatasetReader.o: $(CPPSRC)/DatasetReader.cpp
	g++ $(CCOBJ) -o DatasetReader.o $(CPPSRC)/DatasetReader.cpp
orienteering.o: $(CPPSRC)/orienteering.cpp
	g++ $(CCOBJ) -o orienteering.o $(CPPSRC)/orienteering.cpp
dcvr_dry_run: $(CPPSRC)/dcvr.cpp
	g++ $(CCOBJ) -DIL_STD -I $(CPLEXINCDIR) -I $(CONCERTINCDIR) $(CPPSRC)/dcvr.cpp
iterPCA_dry_run: $(CPPSRC)/iterPCA.cpp
	g++ $(CCOBJ) $(CPPSRC)/iterPCA.cpp

iterPCA: iterPCA.o
	g++ $(CCFLAGS) -o iterPCA.out iterPCA.o
orienteering: orienteering.o iterPCA.o
	g++ $(CCFLAGS) -o orienteering.out orienteering.o iterPCA.o 

test_all: test_iterPCA test_get_path test_cut_path test_orienteering test_dcvr


test_iterPCA: iterPCA.o PcaReader.o $(TESTS)/test_iterPCA.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_iterPCA.out $(TESTS)/test_iterPCA.cpp iterPCA.o PcaReader.o
test_get_path: iterPCA.o orienteering.o $(TESTS)/test_get_path.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_get_path.out $(TESTS)/test_get_path.cpp orienteering.o iterPCA.o
test_cut_path: iterPCA.o orienteering.o $(TESTS)/test_cut_path.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_cut_path.out $(TESTS)/test_cut_path.cpp orienteering.o iterPCA.o

test_orienteering: iterPCA.o orienteering.o DatasetReader.o $(TESTS)/test_orienteering.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_orienteering.out $(TESTS)/test_orienteering.cpp orienteering.o iterPCA.o  DatasetReader.o
test_dcvr: dcvr.o orienteering.o iterPCA.o $(TESTS)/test_dcvr.cpp DatasetReader.o
	g++ $(CCFLAGS) -DIL_STD $(CCLNDIRS) -o $(TESTS)/test_dcvr.out $(TESTS)/test_dcvr.cpp dcvr.o orienteering.o iterPCA.o DatasetReader.o $(CCLNFLAGS)
clean:
	rm -rf *.o $(TESTS)/*.dSYM $(TESTS)/*.out *.out *.dSYM **.log **.json