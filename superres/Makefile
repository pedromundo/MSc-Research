all: converte superre
converte: convert.cpp
	g++ -o converter -Ofast convert.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

superre: super_resolution.cpp 
	g++ -o superres -Ofast super_resolution.cpp -lopencv_core -lopencv_superres -lopencv_highgui -lopencv_videoio -lopencv_cudaoptflow -lopencv_cudacodec -lopencv_cudaimgproc
