cc = g++
name = CG_hwEC
ps_file = ExtraCredit.ps

.PHONY = clean

build: main.cpp draw.cpp
	$(cc) $^ -g -o $(name)

test: build
	@echo "Starting default"
	./$(name) > out.xpm
	@echo "Starting 1"
	./$(name) -f $(ps_file) -a 0 -b 0 -c 500 -d 500 -j 0 -k 0 -o 500 -p 500 > hwEC.xpm
	@echo "Starting 2"
	./$(name) -f $(ps_file) -a 110 -b 0 -c 410 -d 500 -j 0 -k 110 -o 500 -p 410 -s 1 -m 0 -n 0 -r 0 -N 20 > hwEC.xpm
	@echo "Starting 3"
	./$(name) -f $(ps_file) -a 10 -b 10 -c 500 -d 400 -j 10 -k 10 -o 500 -p 400 -s 1.2 -m 30 -n 20 -r 10 -N 20 > hwEC.xpm
	@echo "Starting 4"
	./$(name) -N 4 > hwEC.xpm
	@echo "Starting 5"
	./$(name) -a 110 -b 50 -c 410 -d 500 -j 123 -k 50 -o 379 -p 477 -N 8 > hwEC.xpm

clean:
	-rm -r *.xpm
	-rm $(name)
