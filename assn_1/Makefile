cc = g++
name = CG_hw1

.PHONY = clean

build: main.cpp draw.cpp
	$(cc) $^ -o $(name)

test: build
	@echo "Default"
	./$(name) > out.pbm
	@echo "HW 1"
	./$(name) -s 0.8 -m 85 -n 25 -r 10 > hw1.pbm
	@echo "Scale"
	./$(name) -f hw1.ps -s 0.5 > scale.pbm
	@echo "nRotate"
	./$(name) -f hw1.ps -r -30 > nrotate.pbm
	@echo "Translate"
	./$(name) -f hw1.ps -m 100 -n 100 > translate.pbm
	@echo "World Window"
	./$(name) -f hw1.ps -a 25 -b 50 -c 399 -d 399 > worldwindow.pbm
	@echo "All"
	./$(name) -f hw1.ps -a 25 -b 50 -c 399 -d 399 -r 30 -m 100 -n 100 -s 0.5 > all.pbm

clean:
	-rm -r *.pbm
	-rm $(name)
