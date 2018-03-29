CC = g++
OBJ = Test_utility.o Wire.o

exec: $(OBJ)
	$(CC) -o exec $(OBJ)

%.o: %.cpp
	$(CC) -c $<

