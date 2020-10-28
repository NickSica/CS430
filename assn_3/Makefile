cc = g++
name = CG_hw3
ps_file = hw3_split.ps

.PHONY = clean

build: main.cpp draw.cpp
	$(cc) $^ -g -o $(name)

test: build
	@echo "Starting a"
	./$(name) -f $(ps_file) > hw3_a.pbm
	@echo "Starting b"
	./$(name) -f $(ps_file) -a 0 -b 0 -c 500 -d 500 -j 0 -k 0 -o 500 -p 500 -s 1.0 -m 0 -n 0 -r 0 > hw3_b.pbm
	@echo "Starting c"
	./$(name) -f $(ps_file) -a 50 -b 0 -c 325 -d 500 -j 0 -k 110 -o 480 -p 410 -s 1 -m 0 -n 0 -r 0 > hw3_c.pbm
	@echo "Starting d"
	./$(name) -f $(ps_file) -a 10 -b 10 -c 550 -d 400 -j 10 -k 10 -o 500 -p 400 -s 1.2 -m 6 -n 25 -r 8 > hw3_d.pbm
	@echo "Starting e"
	./$(name) -b 62 -c 500 -d 479 -r 75 -j 139 -o 404 -p 461 -s .85 -m 300 > hw3_e.pbm
	@echo "Starting f"
	./$(name) -a 275 -b 81 -c 550 -d 502 -r -37 -j 123 -k 217 -o 373 -p 467 > hw3_f.pbm
	@echo "Starting g"
	./$(name) -f $(ps_file) -d 301 -c 435 -b 170 -a -100 -r -23 > hw3_g.pbm
	@echo "Starting h"
	./$(name) -f $(ps_file) -a -135 -b -53 -c 633 -d 842 -m -23 -j 101 -p 415 -s 3.6 > hw3_h.pbm

clean:
	-rm -r *.pbm
	-rm $(name)
