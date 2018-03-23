#include <opencv2/opencv.hpp>
#include <sstream>
#include <cstdio>
#include <iostream>

#define NUM_IMAGES 6

using namespace cv;

int save_ply(Mat depth_mat, std::string filename) {
    FILE *fp;

    if ((fp = fopen(filename.c_str(), "w")) == NULL) {
        printf("\nError: while creating file %s", filename);
        return 0;
    }

    int i, j;

    //normals via cross-product
    Mat normal_mat = Mat(depth_mat.size(), CV_32FC3);
    for(int x = 0; x < depth_mat.size().height; ++x)
    {
        for(int y = 0; y < depth_mat.size().width; ++y)
        {
            unsigned char right = 0, left = 0, down = 0, up = 0;
            if(x > 0 && y > 0 && x < depth_mat.size().height-1 && y < depth_mat.size().width-1)
                right = depth_mat.at<Vec3b>(x+1, y)[1], left = depth_mat.at<Vec3b>(x-1, y)[1],
                    down = depth_mat.at<Vec3b>(x, y+1)[1], up = depth_mat.at<Vec3b>(x, y-1)[1];
            

            //threshold (as per Hinterstoisser et al (2011))
            if(abs(depth_mat.at<Vec3b>(x, y)[1] - right) > 20) {
                right = 0;
            }
            if(abs(depth_mat.at<Vec3b>(x, y)[1] - left) > 20) {
                left = 0;
            }
            if(abs(depth_mat.at<Vec3b>(x, y)[1] - down) > 20) {
                down = 0;
            }
            if(abs(depth_mat.at<Vec3b>(x, y)[1] - up) > 20) {
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
            if(z_in_mm != 0 && z_in_mm > 829){
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
            if(z_in_mm != 0 && z_in_mm > 829) {
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
    Mat lr_images[NUM_IMAGES];
    Mat alignment_matrices[NUM_IMAGES];
    Mat template_image;
    Mat hr_image;

    for(size_t i = 0; i < NUM_IMAGES;++i){
        std::ostringstream oss_in;
        oss_in << "cap_depth_" << i << ".png";
        lr_images[i] = imread(oss_in.str(),CV_LOAD_IMAGE_ANYDEPTH);
        oss_in << ".ply";
        save_ply(lr_images[i],oss_in.str());
        lr_images[i].convertTo(lr_images[i],CV_32FC1);

        if(i == 0){
            lr_images[i].copyTo(template_image);
            hr_image = Mat(template_image.size()*2,CV_32FC1);
        }

        //align to first SR view
        findTransformECC(template_image,lr_images[i],alignment_matrices[i], MOTION_AFFINE, \
        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 200, 1E-12));
        std::cout << "Alignment " << i << "-> 0 = " << alignment_matrices[i] << std::endl;

        //upsample
        /*pyrUp(lr_images[i],lr_images[i],lr_images[i].size()*2);*/
        resize(lr_images[i],lr_images[i],lr_images[i].size()*2,0,0,INTER_NEAREST);

        if(i > 0){
            warpAffine(lr_images[i],lr_images[i],alignment_matrices[i],lr_images[i].size(),CV_WARP_INVERSE_MAP);
        }

        //convert to 16bit for visualization and then back
        lr_images[i].convertTo(lr_images[i],CV_16UC1);
        std::ostringstream oss_out;
        oss_out << "cap_depth_" << i << "_registered.png";
        /*lr_images[i] = lr_images[i] * 10;*/
        /*waitKey(30);*/
        imwrite(oss_out.str(),lr_images[i]);
        lr_images[i].convertTo(lr_images[i],CV_32FC1);

        //fuse via averaging
        addWeighted(hr_image,1.0,lr_images[i],(1.0/NUM_IMAGES),0,hr_image);
    }
        resize(hr_image,hr_image,hr_image.size()/2,0,0,INTER_NEAREST);
        hr_image.convertTo(hr_image,CV_16UC1);
        /*hr_image = hr_image * 10;*/
        imwrite("cap_depth_hr.png",hr_image);
        save_ply(hr_image,"cap_depth_hr.ply");
}