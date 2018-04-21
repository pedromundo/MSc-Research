#include <opencv2/opencv.hpp>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <limits>

#define NUM_IMAGES 16
#define RESAMPLE_FACTOR 4

using namespace cv;


//From https://stackoverflow.com/questions/1719070/1719155#1719155
//this will be used to test median fusion instead of average
float median(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

int save_ply(Mat depth_mat, std::string filename, float min_value=std::numeric_limits<float>::min(),\
             float max_value=std::numeric_limits<float>::max()) {
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
            uint16_t right = 0, left = 0, down = 0, up = 0;

            if(x > 0 && y > 0 && x < depth_mat.size().height-1 && y < depth_mat.size().width-1) {
                right = depth_mat.at<uint16_t>(x+1, y);
                left = depth_mat.at<uint16_t>(x-1, y);
                down = depth_mat.at<uint16_t>(x, y+1);
                up = depth_mat.at<uint16_t>(x, y-1);
            }

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

            if(z_in_mm != 0 && z_in_mm >= 829 /*&& z_in_mm <= max_value*/) {
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

            if(z_in_mm != 0 && z_in_mm >= 829 /*&& z_in_mm <= max_value*/) {
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

int main(int argc, char **argv) {
    Mat lr_images[NUM_IMAGES]; //stored as float
    Mat lr_images_upsampled[NUM_IMAGES]; //stored as float
    Mat alignment_matrices[NUM_IMAGES]; //affine transform matrices
    Mat hr_image;

    float global_min = std::numeric_limits<float>::max(), global_max = std::numeric_limits<float>::min();

    //Pre-processing Phase - get minimum and maximum values from the LR images
    //we use this information to remove extreme values from the HR image because
    //some non-linear noise might be added on the resampling and fusion phases
    for(size_t i = 0; i < NUM_IMAGES; ++i) {
        std::ostringstream oss_in;
        oss_in << "cap_depth_" << i << ".png";
        lr_images[i] = imread(oss_in.str(),CV_LOAD_IMAGE_ANYDEPTH);
        lr_images[i].convertTo(lr_images[i],CV_32FC1);

        double local_min, local_max;
        Mat mask = lr_images[i] > 0;

        minMaxLoc(lr_images[i],&local_min,&local_max,NULL,NULL,mask);

        global_min = local_min<global_min?local_min:global_min;
        global_max = local_max>global_max?local_max:global_max;
    }

    printf("Observed Global Min: %.0lf\nObserved Global Max: %.0lf\n",global_min,global_max);

    //Registration Phase - Subpixel registration of all LR images to the first
    for(size_t i = 0; i < NUM_IMAGES; ++i) {
        Mat template_image = lr_images[0]; //first LR image
        findTransformECC(template_image,lr_images[i],alignment_matrices[i], MOTION_AFFINE, \
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 200, 1E-12));
        std::cout << "Alignment " << i << "-> 0 = " << alignment_matrices[i] << std::endl;
    }

    //Upsample + Warp Phase - Upsample all LR images and use registration information
    for(size_t i = 0; i < NUM_IMAGES; ++i) {
        //Warp - Simplified to rigid transform because we aim to have as little
        //translation and rotation between the LR images as possible while still
        //modifying the intrinsics enough to have complementary data
        if(i > 0) {
            warpAffine(lr_images[i],lr_images[i],alignment_matrices[i],\
                       lr_images[i].size(),CV_WARP_INVERSE_MAP);
        }

        //Upsample - can use pyramids or perform a simple scale operation
        //pyrUp(lr_images[i],lr_images_upsampled[i],lr_images[i].size()*RESAMPLE_FACTOR);
        cv::resize(lr_images[i],lr_images_upsampled[i],lr_images[i].size()*RESAMPLE_FACTOR,0,0,INTER_NEAREST);
    }

    //Reconstruction Phase - For now, simply averaging the images, this reduces the
    //impact of the additive noise from the kinect sensor on the HR image (Richardt).
    //Nonlinear noise generated during the other phases will be removed using the
    //previously established limits of the depth info; i.e., we can not generate or
    //reconstruct information outside of the observed volume. This implementation
    //performs poorly in case the images are not very well aligned. In fact, even
    //changing the ECC epsilon from 1E-12 to 1E-06 adversely affected the results
    hr_image = Mat(lr_images_upsampled[0].size(),CV_32FC1);
    for(size_t i = 0; i < NUM_IMAGES; ++i) {
        hr_image = hr_image + lr_images_upsampled[i]/NUM_IMAGES;
        /*addWeighted(hr_image,1.0,lr_images_upsampled[i],(1.0/NUM_IMAGES),0,hr_image);*/
    }

    //Downsample the HR image back to 640x480 to keep it valid within the coordinate
    //system of the Microsoft Kinect sensor
    cv::resize(hr_image,hr_image,hr_image.size()/RESAMPLE_FACTOR,0,0,INTER_AREA);
    /*pyrDown(hr_image,hr_image,hr_image.size()/RESAMPLE_FACTOR);*/
    hr_image.convertTo(hr_image,CV_16UC1);
    imwrite("cap_depth_hr.png",hr_image);
    save_ply(hr_image,"cap_depth_hr.ply",global_min,global_max);
}