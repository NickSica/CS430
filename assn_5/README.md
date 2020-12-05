# Assignment 5 
The program takes in a SMF file and several command line parameters and outputs a PPM file of the rendered 3D objects after the transformations and clippings.  
The language used was C++ programmed originally on a linux machine.  
The compiler was g++ and it is compiled by invoking the g++ and linking the files main.cpp and draw.cpp as well as eigen if it isn't in your path:
"g++ main.cpp draw.cpp -o CG_hw5". If it is being run on tux you need to link eigen: "g++ main.cpp draw.cpp -I/site/local/ -o CG_hw5". 
The file containing main is main.cpp.
