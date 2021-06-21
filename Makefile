CCOPT = -m64 -O2 -std=c++11 -g -fPIC -fno-strict-aliasing -fexceptions $(DEBUG_FLAG)
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
#  -DNDEBUG

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


Path.o: $(CPPSRC)/Path.cpp
	g++ -c $(CCFLAGS)  -o Path.o $(CPPSRC)/Path.cpp
dcvr.o: $(CPPSRC)/dcvr.cpp
	g++ $(CCOBJ) -DIL_STD -I $(CPLEXINCDIR) -I $(CONCERTINCDIR) -o dcvr.o $(CPPSRC)/dcvr.cpp
iterPCA.o: $(CPPSRC)/iterPCA.cpp
	g++ $(CCOBJ) -o iterPCA.o $(CPPSRC)/iterPCA.cpp
heuristics.o: $(CPPSRC)/heuristics.cpp
	g++ $(CCOBJ) -o heuristics.o $(CPPSRC)/heuristics.cpp
PcaReader.o: $(CPPSRC)/PcaReader.cpp
	g++ $(CCOBJ) -o PcaReader.o $(CPPSRC)/PcaReader.cpp
DatasetReader.o: $(CPPSRC)/DatasetReader.cpp
	g++ $(CCOBJ) -o DatasetReader.o $(CPPSRC)/DatasetReader.cpp
rooted_orienteering.o: $(CPPSRC)/rooted_orienteering.cpp
	g++ $(CCOBJ) -o rooted_orienteering.o $(CPPSRC)/rooted_orienteering.cpp
cycle_orienteering.o: $(CPPSRC)/cycle_orienteering.cpp
	g++ $(CCOBJ) -o cycle_orienteering.o $(CPPSRC)/cycle_orienteering.cpp
p2p_orienteering.o: $(CPPSRC)/p2p_orienteering.cpp
	g++ $(CCOBJ) -o p2p_orienteering.o $(CPPSRC)/p2p_orienteering.cpp
dcvr_dry_run: $(CPPSRC)/dcvr.cpp
	g++ $(CCOBJ) -DIL_STD -I $(CPLEXINCDIR) -I $(CONCERTINCDIR) $(CPPSRC)/dcvr.cpp
iterPCA_dry_run: $(CPPSRC)/iterPCA.cpp
	g++ $(CCOBJ) $(CPPSRC)/iterPCA.cpp

iterPCA: iterPCA.o
	g++ $(CCFLAGS) -o iterPCA.out iterPCA.o
rooted_orienteering: rooted_orienteering.o iterPCA.o Path.o
	g++ $(CCFLAGS) -o rooted_orienteering.out rooted_orienteering.o iterPCA.o  Path.o


run_tests: test_all
	$(TESTS)/test_rooted_orienteering.out -D 4 < datasets/small-sample.txt
	$(TESTS)/test_cycle_orienteering.out -D 4 < datasets/small-sample.txt
	$(TESTS)/test_dcvr.out -D 4 < datasets/small-sample.txt


test_all: test_iterPCA test_rooted_orienteering test_cycle_orienteering test_dcvr

test_iterPCA: iterPCA.o PcaReader.o $(TESTS)/test_iterPCA.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_iterPCA.out $(TESTS)/test_iterPCA.cpp iterPCA.o PcaReader.o
test_get_path: iterPCA.o rooted_orienteering.o $(TESTS)/test_get_path.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_get_path.out $(TESTS)/test_get_path.cpp rooted_orienteering.o iterPCA.o
test_cut_path: iterPCA.o rooted_orienteering.o $(TESTS)/test_cut_path.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_cut_path.out $(TESTS)/test_cut_path.cpp rooted_orienteering.o iterPCA.o

test_rooted_orienteering: iterPCA.o rooted_orienteering.o DatasetReader.o heuristics.o Path.o $(TESTS)/test_rooted_orienteering.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_rooted_orienteering.out $(TESTS)/test_rooted_orienteering.cpp rooted_orienteering.o iterPCA.o  DatasetReader.o heuristics.o Path.o
test_cycle_orienteering: iterPCA.o cycle_orienteering.o DatasetReader.o heuristics.o Path.o $(TESTS)/test_cycle_orienteering.cpp
	g++ $(CCFLAGS) -o $(TESTS)/test_cycle_orienteering.out $(TESTS)/test_cycle_orienteering.cpp  cycle_orienteering.o iterPCA.o  DatasetReader.o heuristics.o Path.o
test_dcvr: dcvr.o rooted_orienteering.o iterPCA.o heuristics.o $(TESTS)/test_dcvr.cpp DatasetReader.o Path.o
	g++ $(CCFLAGS) -DIL_STD $(CCLNDIRS) -o $(TESTS)/test_dcvr.out $(TESTS)/test_dcvr.cpp dcvr.o rooted_orienteering.o iterPCA.o DatasetReader.o heuristics.o Path.o $(CCLNFLAGS)
clean:
	rm -rf *.o $(TESTS)/*.dSYM $(TESTS)/*.out *.out *.dSYM **.log **.json