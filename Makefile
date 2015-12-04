CXX = g++
EXE = Orienteering
OBJ = orienteering.o
RM = rm -f

$(EXE):$(OBJ)
	$(CXX) -o $(EXE) $(OBJ)

orienteering.o:orienteering.cpp
	$(CXX) -c orienteering.cpp

.PHONY:clean
clean:
	$(RM) $(EXE) $(OBJ)
