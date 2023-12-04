yash:	main.o add.o sub.o  
	gcc -o mai add.o sub.o main.o 
main.o:
	gcc -c main.c 
add.o:
	gcc -c add.c
sub.o:
	gcc -c sub.c
clean:
	rm -rf *.o mai static.a dynamic.so a.out dy *.so *.a
static:
	ar -cr static.a add.o sub.o 
dyna:
	gcc -o libyasw.so add.o sub.o -shared -fPIC 
