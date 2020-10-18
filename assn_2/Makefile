cc = g++
name = CG_hw2

.PHONY = clean

build: main.cpp draw.cpp
	$(cc) $^ -g -o $(name)

test: build
	@echo "Ex 1"
	./$(name) -f hw2_a.ps -s 1.5 > hw2_ex1.pbm
	@echo "Ex 2"
	./$(name) -f hw2_a.ps -m -250 -n -200 > hw2_ex2.pbm
	@echo "Ex 3"
	./$(name) -f hw2_b.ps -a 170 -b 100 -c 270 -d 400 > hw2_ex3.pbm
	@echo "Ex 4"
	./$(name) -f hw2_b.ps -s 2 > hw2_ex4.pbm
	@echo "Ex 5"
	./$(name) -f hw2_c.ps -a 200 -b 100 -c 375 -d 400  > hw2_ex5.pbm
	@echo "Ex 6"
	./$(name) -f hw2_c.ps -a 275 -b 100 -c 550 -d 502 > hw2_ex6.pbm
	@echo "Ex 7"
	./$(name) -f hw2_b.ps -d 270 -c 435 -b 170 -a 100 -r 17 > hw2_ex7.pbm
	@echo "Ex 8"
	./$(name) -f hw2_b.ps -a -135 -b -53 -c 633 -d 442  > hw2_ex8.pbm
	@echo "Ex 9"
	./$(name) -f hw2_c.ps -a -150 -b -475 -c 123 -d -65 > hw2_ex9.pbm

clean:
	-rm -r *.pbm
	-rm $(name)
