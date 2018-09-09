#include <opencv2/opencv.hpp>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <limits>
#include <numeric>

#define SR_SIZE 16
#define RESAMPLE_FACTOR 4

using namespace cv;

//Adapted from https://stackoverflow.com/questions/7616511/
//I really should get more acquainted with C++'s stl1
float mean(std::vector<float> &v)
{
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    return mean;
}

//From https://stackoverflow.com/questions/1719070/1719155#1719155
//this will be used to test median fusion instead of average
float median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

int save_ply(Mat depth_mat, std::string filename, float min_value = std::numeric_limits<float>::min(),
             float max_value = std::numeric_limits<float>::max())
{
    FILE *fp;

    if ((fp = fopen(filename.c_str(), "w")) == NULL)
    {
        printf("\nError: while creating file %s", filename.c_str());
        return 0;
    }

    int i, j;

    //normals via cross-product
    Mat normal_mat = Mat(depth_mat.size(), CV_32FC3);
    for (int i = 0; i < depth_mat.size().height; ++i)
    {
        for (int j = 0; j < depth_mat.size().width; ++j)
        {
            uint16_t right = 0, left = 0, down = 0, up = 0;

            if (i > 0 && j > 0 && i < depth_mat.size().height - 1 && j < depth_mat.size().width - 1)
            {
                right = depth_mat.at<uint16_t>(i + 1, j);
                left = depth_mat.at<uint16_t>(i - 1, j);
                down = depth_mat.at<uint16_t>(i, j + 1);
                up = depth_mat.at<uint16_t>(i, j - 1);
            }

            //threshold (as per Hinterstoisser et al (2011))
            if (abs(depth_mat.at<uint16_t>(i, j) - right) > 20)
            {
                right = 0;
            }
            if (abs(depth_mat.at<uint16_t>(i, j) - left) > 20)
            {
                left = 0;
            }
            if (abs(depth_mat.at<uint16_t>(i, j) - down) > 20)
            {
                down = 0;
            }
            if (abs(depth_mat.at<uint16_t>(i, j) - up) > 20)
            {
                up = 0;
            }

            double dzdx = (right - left) / 2.0;
            double dzdy = (down - up) / 2.0;

            Vec3d d(-dzdx, -dzdy, 1.0);
            Vec3d n = normalize(d);

            normal_mat.at<Vec3f>(i, j) = n;
        }
    }

    uint32_t num_vertices = 0;
    //iterate to count the number of vertices
    for (i = 0; i < depth_mat.size().height; i++)
    {
        for (j = 0; j < depth_mat.size().width; j++)
        {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i, j);

            if (z_in_mm != 0 && z_in_mm >= min_value && z_in_mm <= max_value)
            {
                ++num_vertices;
            }
        }
    }

    //then a second time to write the .ply file
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", num_vertices);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property float nx\n");
    fprintf(fp, "property float ny\n");
    fprintf(fp, "property float nz\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    for (i = 0; i < depth_mat.size().height; i++)
    {
        for (j = 0; j < depth_mat.size().width; j++)
        {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i, j);
            double fx = 572.882768, fy = 542.739980, cx = 314.649173, cy = 240.160459;
            if (z_in_mm != 0 && z_in_mm >= min_value && z_in_mm <= max_value)
            {
                double vx = z_in_mm * (j - cx) * (1/fx);
                double vy = -z_in_mm * (i - cy) * (1/fy);
                Vec3f normal = normal_mat.at<Vec3f>(i, j);
                fprintf(fp, "%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf %d %d %d\n",
                        (double)vx, (double)vy, (double)-z_in_mm, (double)normal[0],
                        (double)normal[1], (double)normal[2], 128, 128, 128);
            }
        }
    }

    fclose(fp);
    fflush(fp);

    printf("%s saved!\n", filename.c_str());
    fflush(stdout);

    return 1;
}

int main(int argc, char **argv)
{
    std::string capture_name = "cap";
    int view_angle = 0;

    if(argc > 1){
        capture_name = std::string(argv[1]);
    }
    if(argc > 2){
        view_angle = std::stoi(std::string(argv[2]));
    }

    std::cout << capture_name << " " << view_angle << std::endl;

    Mat lr_images[SR_SIZE];           //stored as float
    Mat lr_images_upsampled[SR_SIZE]; //stored as float
    Mat alignment_matrices[SR_SIZE];  //affine transform matrices
    Mat hr_image;

    float global_min = std::numeric_limits<float>::max(), global_max = std::numeric_limits<float>::min();

    //Pre-processing Phase - get minimum and maximum values from the LR images
    //we use this information to remove extreme values from the HR image because
    //some non-linear noise might be added on the resampling and fusion phases
    for (size_t i = 0; i < SR_SIZE; ++i)
    {
        std::ostringstream oss_in;
        oss_in << capture_name << "_burst_" << view_angle << "_" << i << ".png";
        lr_images[i] = imread(oss_in.str(), CV_LOAD_IMAGE_ANYDEPTH);
        lr_images[i].convertTo(lr_images[i], CV_32FC1);
        alignment_matrices[i] = Mat::eye(2, 3, CV_32F);

        double local_min, local_max;
        Mat mask = lr_images[i] > 0;

        minMaxLoc(lr_images[i], &local_min, &local_max, NULL, NULL, mask);

        global_min = local_min < global_min ? local_min : global_min;
        global_max = local_max > global_max ? local_max : global_max;
    }

    printf("Observed Global Min: %.0lf\nObserved Global Max: %.0lf\n", global_min, global_max);

    //Registration Phase - Subpixel registration of all LR images to the first
    #pragma omp parallel for
    for (size_t i = 0; i < SR_SIZE; ++i)
    {
        Mat template_image = lr_images[0]; //first LR image
        findTransformECC(template_image, lr_images[i], alignment_matrices[i], MOTION_AFFINE,
                         TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 200, 1E-12));
        std::cout << "Alignment " << i << "-> 0 = " << alignment_matrices[i] << std::endl;
    }

    //Upsample + Warp Phase - Upsample all LR images and use registration information
    //technically the resize operation should happen before the warpAffine, but this
    //is not working in this version of the program, whilst it worked on an older
    //version, ignoring this for now since the results are good
    for (size_t i = 0; i < SR_SIZE; ++i)
    {
        //Warp - Simplified to rigid transform because we aim to have as little
        //translation and rotation between the LR images as possible while still
        //modifying the intrinsics enough to have complementary data
        if (i > 0)
        {
            warpAffine(lr_images[i], lr_images[i], alignment_matrices[i], lr_images[i].size(), CV_WARP_INVERSE_MAP);
        }

        //Upsample - can use pyramids or perform a simple scale operation
        //results were exactly the same, still gotta find out why
        cv::resize(lr_images[i], lr_images_upsampled[i], lr_images[i].size() * RESAMPLE_FACTOR, 0, 0, INTER_NEAREST);
    }

    //Reconstruction Phase - For now, simply averaging the images, this reduces the
    //impact of the additive noise from the kinect sensor on the HR image (Richardt).
    //Nonlinear noise generated during the other phases will be removed using the
    //previously established limits of the depth info; i.e., we can not generate or
    //reconstruct information outside of the observed volume. This implementation
    //performs poorly in case the images are not very well aligned. In fact, even
    //changing the ECC epsilon from 1E-12 to 1E-06 adversely affected the results
    /*hr_image = Mat(lr_images_upsampled[0].size(),CV_32FC1);
    for(size_t i = 0; i < SR_SIZE; ++i) {
        addWeighted(hr_image,1.0,lr_images_upsampled[i],(1.0/SR_SIZE),0,hr_image);
    }*/

    //New approach -- zero-elimination (ZE) averaging
    hr_image = Mat(lr_images_upsampled[0].size(), CV_32FC1);
    for (int i = 0; i < hr_image.size().height; i++)
    {
        for (int j = 0; j < hr_image.size().width; j++)
        {
            std::vector<float> candidate_pixels;
            for (int k = 0; k < SR_SIZE; ++k)
            {
                float depth = lr_images_upsampled[k].at<float>(i, j);
                if (depth > 0)
                {
                    candidate_pixels.push_back(depth);
                }
            }
            float &pix = hr_image.at<float>(i, j);
            pix = mean(candidate_pixels);
        }
    }

    //Downsample the HR image back to 640x480 to keep it valid within the coordinate
    //system of the Microsoft Kinect sensor - lanczos removes some artifacts with
    //no apparent downsides
    cv::resize(hr_image, hr_image, hr_image.size() / RESAMPLE_FACTOR, 0, 0, INTER_LANCZOS4);
    hr_image.convertTo(hr_image, CV_16UC1);
    std::ostringstream oss_out;
    oss_out << capture_name << "_srdepth_" << view_angle << ".png";
    imwrite(oss_out.str(),hr_image);
    oss_out.str(std::string());
    oss_out << capture_name << "_srmesh_" << view_angle << ".ply";
    save_ply(hr_image, oss_out.str(), global_min, global_max);
}