cc = g++
name = CG_hw5
smf_file = bound-sprellpsd.smf

.PHONY = clean

build: main.cpp draw.cpp
	$(cc) $^ -g -o $(name)

test: build
	@echo "Starting default"
	./$(name) > out.ppm
	@echo "Starting cow b"
	./$(name) -f bound-cow.smf -F 0.1 > hw5_b.ppm
	@echo "Starting cow c"
	./$(name) -f bound-cow.smf -F 0.1 -B -0.18 > hw5_c.ppm
	@echo "Starting d"
	./$(name) -f bound-bunny_1k.smf -g bound-cow.smf -i bound-sprtrd.smf -u -.8 -U .8 -v -.8 -V .8 > hw5_d.ppm
	@echo "Starting e"
	./$(name) -f bound-bunny_1k.smf -g bound-cow.smf -i bound-sprtrd.smf -q .4 -r .2 -w 1 -u -.6 -v -.6 -U .6 -V .6 -P > hw5_e.ppm
	@echo "Starting f"
	./$(name) -j 334 -k 24 -o 449 -p 402 -f bound-sprellpsd.smf > hw5_f.ppm
	@echo "Starting g"
	./$(name) -u -1.0 -U 0.8 -v -0.9 -V 1.0 -g bound-bunny_1k.smf -f bound-sprtrd.smf -j 34 -k 104 -o 149 -p 472 > hw5_g.ppm
	@echo "Starting h"
	./$(name) -u -1.0 -U 0.8 -v -0.9 -V 1.0 -g bound-bunny_1k.smf -f bound-sprtrd.smf -j 34 -k 104 -o 149 -p 472 -Q -1.1 -R 0.9 -W 0.6 > hw5_h.ppm

clean:
	-rm -r *.ppm
	-rm $(name)
