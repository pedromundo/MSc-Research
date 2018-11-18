all:
	gcc -c -w -Ofast rply.c
	g++ -c -w -Ofast main.cpp
	g++ -c -w -Ofast initShaders.cpp

	g++ -w -Ofast -o mapping -flto rply.o main.o initShaders.o -lGL -lGLU -lGLEW -lglut -lSOIL