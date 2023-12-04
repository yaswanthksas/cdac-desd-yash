CC = gcc 
main: main.o add.o sub.o 
	$(CC) -o main main.o add.o sub.o 
main.o:
	$(CC) -c main.c
add.o:
	$(CC) -c add.c
sub.o:
	$(CC) -c sub.c 
clean:
	rm -rf *.o main 
makelib:
	ar -crv arithlib.a add.o sub.o 

