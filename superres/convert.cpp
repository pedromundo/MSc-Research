#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int save_ply(Mat depth_mat, string filename) {
    FILE *fp;

    if ((fp = fopen(filename.c_str(), "w")) == NULL) {
        printf("\nError: while creating file %s", filename);
        return 0;
    }

    int i, j;

    //simple normals via cross-product
    Mat normal_mat = Mat(depth_mat.size(), CV_32FC3);
    for(int x = 1; x < depth_mat.rows-1; ++x)
    {
        for(int y = 1; y < depth_mat.cols-1; ++y)
        {
            uint16_t right = depth_mat.at<uint16_t>(x+1, y), left = depth_mat.at<uint16_t>(x-1, y),
                     down = depth_mat.at<uint16_t>(x, y+1), up = depth_mat.at<uint16_t>(x, y-1);

            //threshold (as per Hinterstoisser et al (2011))
            if(abs(depth_mat.at<uint16_t>(x, y) - right) > 20) {
                right = 0;
            }
            if(abs(depth_mat.at<uint16_t>(x, y) - left) > 20) {
                left = 0;
            }
            if(abs(depth_mat.at<uint16_t>(x, y) - down) > 20) {
                down = 0;
            }
            if(abs(depth_mat.at<uint16_t>(x, y) - up) > 20) {
                up = 0;
            }

            double dzdx = (right - left) / 2.0;
            double dzdy = (down - up) / 2.0;

            Vec3d d(-dzdx, -dzdy, 1.0);
            Vec3d n = normalize(d);

            normal_mat.at<Vec3f>(x, y) = n;
        }
    }

    uint32_t num_vertices = 0;
    //iterate to count the number of vertices
    for (i = 0; i < depth_mat.size().height; i++) {
        for (j = 0; j < depth_mat.size().width; j++) {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i,j);
            if(z_in_mm != 0){
                ++num_vertices;
            }
        }
    }

    //then a second time to write the .ply file
    fprintf(fp,"ply\n");
    fprintf(fp,"format ascii 1.0\n");
    fprintf(fp,"element vertex %d\n",num_vertices);
    fprintf(fp,"property float x\n");
    fprintf(fp,"property float y\n");
    fprintf(fp,"property float z\n");
    fprintf(fp,"property float nx\n");
    fprintf(fp,"property float ny\n");
    fprintf(fp,"property float nz\n");
    fprintf(fp,"property uchar red\n");
    fprintf(fp,"property uchar green\n");
    fprintf(fp,"property uchar blue\n");
    fprintf(fp,"end_header\n");

    for (i = 0; i < depth_mat.size().height; i++) {
        for (j = 0; j < depth_mat.size().width; j++) {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i,j);
            double cx = 313.68782938, cy = 259.01834898, fx_inv = 1 / 526.37013657, fy_inv = fx_inv;
            if(z_in_mm != 0) {
                double vx = z_in_mm * (j - cx) * fx_inv;
                double vy = -z_in_mm * (i - cy) * fy_inv;
                Vec3f normal = normal_mat.at<Vec3f>(i,j);
                fprintf(fp, "%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf %d %d %d\n",
                        (double) vx, (double) vy, (double) -z_in_mm, (double) normal[0],
                        (double) normal[1], (double) normal[2], 128, 128, 128);
            }
        }
    }

    fclose(fp);
    fflush(fp);

    printf("%s saved!\n", filename.c_str());
    fflush(stdout);

    return 1;
}

int main(int argc, char **argv){
    /*read 16 bit 1 channel and convert to 8 bit 3 channels
    8 most significant bits of the 16 bit píxels are stored 
    in the first channel and the 8 least significant ones are
    stored in the 2nd channel - this is meant to improve
    compatibility with everything...*/

    //C to convert R to restore
    char operation = 'c';
    string out_suffix = "_cvt";

    string image_name;

    std::ostringstream oss;

    if(argc > 1){
        image_name = string(argv[1]);
    }

    if(argc > 2){
        operation = argv[2][0];
    }

    Mat input;
    Mat output;

    if(operation == 'c' || operation == 'C'){
        out_suffix = "_cvt";
        input = imread(image_name,CV_LOAD_IMAGE_ANYDEPTH);
        output = Mat(input.size(),CV_8UC3);
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
    }else if(operation == 'r' || operation == 'R'){
        out_suffix = "_res";
        input = imread(image_name,CV_LOAD_IMAGE_COLOR);
        output = Mat(input.size(),CV_16UC1);
        for(size_t i=0;i < input.size().width;++i){
            for(size_t j=0;j < input.size().height;++j){
                Vec3b input_pixel = input.at<Vec3b>(j,i);
                uint16_t &output_pixel = output.at<uint16_t>(j,i);
                output_pixel = 0;
                if(input_pixel[0] != 0 || input_pixel[1] != 0){
                    output_pixel = output_pixel | input_pixel[1];
                    output_pixel = output_pixel << 8;
                    output_pixel = output_pixel | input_pixel[0];
                    printf("%d %d %ld\n",input_pixel[0],input_pixel[1],output_pixel);
                }
            }
        }
    }else if(operation == 'm' || operation == 'M'){
        out_suffix = "_mesh";
        input = imread(image_name,CV_LOAD_IMAGE_ANYDEPTH);        
        oss << image_name.substr(0, (image_name.length() - 4)) << out_suffix << ".ply";
        save_ply(input, oss.str());
        return 1;
    }else{
        printf("Invalid operation requested (2nd argument must be either C - convert or R - restore.)\n");
    }

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    oss << image_name.substr(0, (image_name.length() - 4)) << out_suffix << ".png";
    imwrite(oss.str(),output,compression_params);
}