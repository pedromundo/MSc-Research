all: converter superres
converter: convert.cpp
	g++ -o converter -Ofast convert.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

superres: super_resolution.cpp
	g++ -o superres super_resolution.cpp -lopencv_core -lopencv_superres -lopencv_highgui -lopencv_videoio -lopencv_cudaoptflow -lopencv_cudacodec -lopencv_cudaimgproc
