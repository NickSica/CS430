cc = g++
name = CG_hw4
smf_file = bound-lo-sphere.smf

.PHONY = clean

build: main.cpp draw.cpp
	$(cc) $^ -g -o $(name)

test: build
	@echo "Starting default"
	./$(name) > default.pbm
	@echo "Starting circle 1"
	./$(name) -f $(smf_file) -x -1.0 -z 0.5 -q 1.0 -w -0.5 > circle_1.pbm
	@echo "Starting circle 2"
	./$(name) -f $(smf_file) -q 1.0 -w -1.0 > circle_2.pbm
	@echo "Starting circle 3"
	./$(name) -f $(smf_file) -x -4.0 -z 5.0 -q 1.0 -w -0.5 > circle_3.pbm
	@echo "Starting circle 4"
	./$(name) -f $(smf_file) -k 125 -p 375 -q 1.0 -u -1.4 -U 1.4 > circle_4.pbm
	@echo "Switching to bunny"
	$(eval smf_file=bound-bunny_200.smf)
	@echo "Starting bunny f"
	./$(name) -f $(smf_file) -j 100 -k 50 -o 400 -p 450 -x 0.5 -y 0.2 -z 1.0 -X 0.2 -Y -0.2 -Z 0.3 -q -3.0 -r -2.0 -w 1.0 -Q 3.0 -R -2 -W -4.0 -u -.5 -v -.9 -U 1.2 -V .8 -P > hw4_f.pbm
	@echo "Starting bunny g"
	./$(name) -f $(smf_file) -j 100 -k 50 -o 400 -p 450 -x 0.5 -y 0.2 -z 1.0 -X 0.2 -Y -0.2 -Z 0.3 -q -3.0 -r -2.0 -w 1.0 -Q 3.0 -R -2 -W -4.0 -u -.5 -v -.9 -U 1.2 -V .8 > hw4_g.pbm
	@echo "Switching to cow"
	$(eval smf_file=bound-cow.smf)
	@echo "Starting cow default"
	./$(name) -f $(smf_file) > cow_default.pbm
	@echo "Starting cow parallel"
	./$(name) -f $(smf_file) -P > cow_parallel.pbm
	@echo "Starting cow jkopP"
	./$(name) -f $(smf_file) -j 0 -k 30 -o 275 -p 305 -P > cow_jkopP.pbm
	@echo "Starting cow x1.5"
	./$(name) -f $(smf_file) -x 1.5 > cow_x1.5.pbm
	@echo "Starting cow xyzPar"
	./$(name) -f $(smf_file) -x 4.75 -y -3.25 -z 3.3 -P > cow_xyzPar.pbm
	@echo "Starting cow XYZ"
	./$(name) -f $(smf_file) -X 0.25 -Y -0.15 -Z 0.3 > cow_XYZ.pbm
	@echo "Starting cow XYZuvP"
	./$(name) -f $(smf_file) -X 0.35 -Y -0.3 -Z 0.3 -u -0.35 -v -0.35 -P > cow_XYZuvP.pbm
	@echo "Starting cow XYZjkop"
	./$(name) -f $(smf_file) -X 0.25 -Y -0.15 -Z 0.3 -j 103 -k 143 -o 421 -p 379 > cow_XYZjkop.pbm
	@echo "Starting cow XYZuvjkopP"
	./$(name) -f $(smf_file) -X 0.35 -Y -0.3 -Z 0.3 -u -0.35 -v -0.35 -j 43 -k 71 -o 201 -p 402 -P > cow_XYZuvjkopP.pbm
	@echo "Starting cow qrw"
	./$(name) -f $(smf_file) -q -1 -r 1.5 -w -2.0 > cow_q_r_w.pbm
	@echo "Starting cow QRW"
	./$(name) -f $(smf_file) -Q 1.5 -R 1 -W .4 > cow_QRW.pbm
	@echo "Starting cow uvUV"
	./$(name) -f $(smf_file) -u -1.5 -v -0.9 -U 1.2 -V 0.7 > cow_uvUV.pbm

clean:
	-rm -r *.pbm
	-rm $(name)
