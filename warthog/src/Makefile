WARTHOG_SRC = $(wildcard domains/*.cpp) $(wildcard util/*.cpp) \
			  $(wildcard search/*.cpp) $(wildcard experimental/*.cpp) \
			  $(wildcard heuristics/*.cpp) $(wildcard jps/*.cpp) \
			  $(wildcard contraction/*.cpp) $(wildcard label/*.cpp) \
			  $(wildcard memory/*.cpp) $(wildcard mapf/*.cpp) $(wildcard sys/*.cpp) 
			  

WARTHOG_OBJ = $(WARTHOG_SRC:.cpp=.o)

D_WARTHOG_INCLUDES = -I./ -I./domains -I./util -I./search -I./experimental \
					 -I./heuristics -I./jps -I./contraction -I./label \
					 -I./memory -I./mapf -I./sys

D_INCLUDES = -I/usr/include -I/usr/local/include $(D_WARTHOG_INCLUDES)
D_LIBS = -L/usr/local/lib -L./lib

CFLAGS = -std=c++11 -pedantic -pthread -Wall -Wno-strict-aliasing \
			-Wno-long-long -Wno-deprecated -Wno-deprecated-declarations \
			-Wno-constant-logical-operand -Werror 
FAST_CFLAGS = -O3 -DNDEBUG
DEV_CFLAGS = -g -ggdb -O0 -gdwarf-4 
#DEV_CFLAGS = -g -ggdb -O0 -gdwarf-4 -fvar-tracking-assignments
PROFILE_CFLAGS = $(DEV_CFLAGS) -DNDEBUG 

ifeq ("$(findstring Darwin, "$(shell uname -s)")", "Darwin")
  CC = clang++
  CFLAGS += -DOS_MAC
else
  CC = clang++
  ifeq ("$(findstring Linux, "$(shell uname -s)")", "Linux")
    D_LIBS += -lrt
  endif
endif


.PHONY: all
all: fast

.PHONY: fast
fast: CFLAGS += $(FAST_CFLAGS) $(D_INCLUDES) 
fast: main extras__

.PHONY: dev 
dev: CFLAGS += $(DEV_CFLAGS) $(D_INCLUDES) 
dev: main extras__

.PHONY: profile
profile: CFLAGS += $(PROFILE_CFLAGS) $(D_INCLUDES) 
profile: main extras__

.PHONY: extras
extras: CFLAGS += $(DEV_CFLAGS) $(D_INCLUDES) 
extras: extras__

.PHONY: release
release: CFLAGS += $(FAST_CFLAGS) $(D_INCLUDES) 
release: fast extras__

.PHONY: x
x: extras

.PHONY: tags
tags:
	ctags -R .
	#find . | grep -E "\.((h(pp)*\$$)|(c(pp)+\$$))"	> cscope.in
	#cscope -b -i cscope.in

.PHONY: clean
clean:
	@rm -rf ./bin/*
	@-$(RM) ./lib/*
	@-$(RM) $(WARTHOG_OBJ:.o=.d)
	@-$(RM) $(WARTHOG_OBJ)

.PHONY: main
main: makedirs warthog 
	@echo "linking..."
	$(CC) programs/warthog.cpp -o ./bin/warthog -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/roadhog.cpp -o ./bin/roadhog -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)

.PHONY: extras__
extras__: makedirs warthog 
	@echo "linking..."
	$(CC) programs/convert.cpp -o ./bin/convert -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/ch.cpp -o ./bin/ch -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/labelmaker.cpp -o ./bin/labelmaker -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/dimacs2metis.cpp -o ./bin/dimacs2metis -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/grid2dimacs.cpp -o ./bin/grid2dimacs -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/tests.cpp -o ./bin/tests -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/closure.cpp -o ./bin/closure -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) programs/fix_dimacs_arc_weights.cpp -o ./bin/fix_dimacs_arc_weights -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)

.PHONY: makedirs
makedirs:
	@echo "### Creating output directories ###"
	@$(shell mkdir ./bin)
	@$(shell mkdir ./lib)

.PHONY: warthog
warthog: $(WARTHOG_OBJ)
	@echo "###  Archiving object files ###"
	@echo "sources "$(WARTHOG_SRC)
	ar -crs lib/lib$(@).a $(WARTHOG_OBJ)
	#rm lib/lib$(@).a

-include $(WARTHOG_OBJ:.o=.d)

$(WARTHOG_OBJ): 
	@echo "compiling..."
	$(CC) -c $(@:.o=.cpp) $(CFLAGS) -MM -MP -MT $(dir $(@))$(notdir $(@)) -MF $(@:.o=.d) 
	$(CC) -c $(@:.o=.cpp) -o $(@) $(CFLAGS) 

