#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv){
    /*read 16 bit 1 channel and convert to 8 bit 3 channels
    8 most significant bits of the 16 bit píxels are stored 
    in the first channel and the 8 least significant ones are
    stored in the 2nd channel - this is meant to improve
    compatibility with everything...*/

    string image_name;

    std::ostringstream oss;

    if(argc > 1){
        image_name = string(argv[1]);
    }

    Mat input = imread(image_name,CV_LOAD_IMAGE_ANYDEPTH);
    Mat output = Mat(input.size(),CV_8UC3);
    for(size_t i=0;i < input.size().width;++i){
        for(size_t j=0;j < input.size().height;++j){
            uint16_t input_pixel = input.at<uint16_t>(j,i);
            Vec3b &output_pixel = output.at<Vec3b>(j,i);
            if(input_pixel != 0){
                output_pixel[0] = (unsigned char)((input_pixel & 0xFF00) >> 8);
                output_pixel[1] = (unsigned char)(input_pixel & 0x00FF);
                output_pixel[2] = 0;
            }else{
                output_pixel[0] = 0;
                output_pixel[1] = 0;
                output_pixel[2] = 0;
            }
        }
    }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    oss << image_name.substr(0, (image_name.length() - 4)) << "_cvt" << ".png";
    imwrite(oss.str(),output,compression_params);
}