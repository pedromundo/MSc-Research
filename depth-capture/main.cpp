#include <libfreenect_sync.h>
#include <opencv2/opencv.hpp>
#include <sstream>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <omp.h>
#include <unistd.h>

//Escape closes the application
#define CLOSE 27
//Image capture keys
#define DEPTH 'q'
#define COLOR 'w'
#define BURST 'e'
//Mesh capture keys
#define DEPTH_MESH 'Q'
//Perform all captures
#define ALL 'A'

//Constants
#define SR_SIZE 16

using namespace cv;

static unsigned int count_depth = 0;
static unsigned int count_burst = 0;
static unsigned int count_color = 0;
static unsigned int count_depth_mesh = 0;

static std::string capture_name = "estatua";
static int capture_step = 20;

//Depth thresholds in mm
int depth_min = 0;
int depth_max = 850;
int left_plane = 0;
int right_plane = 640;
int top_plane = 0;
int bottom_plane = 480;

//OpenCV variables
static Size kinect;

inline double lerp(double a, double b, double f)
{
    return (a * (1.0 - f)) + (b * f);
}

// ---------------------------------------------------------
// Name:		show_depth
// ---------------------------------------------------------
// Description:	transform a depth frame information into color
// 				and show as a video
// ---------------------------------------------------------

void show_depth (uint16_t *depth) {
    Mat depth_mat = Mat(kinect,CV_16UC1, depth);
    Mat depth_mat_tmp = Mat(kinect, CV_8UC3);
    int i, j;
    for ( i = 0; i < kinect.height; i++)
        for ( j = 0; j < kinect.width; j++) {
            uint16_t depth_in_mm = depth_mat.at<uint16_t>(i,j);
            if (depth_in_mm != 0 && depth_in_mm >= depth_min && depth_in_mm <= depth_max && \
                i >= top_plane && i <= bottom_plane && j >= left_plane && j <= right_plane) {
                uint16_t color = (uint16_t) lerp(255.0,32.0,(double) depth_in_mm/FREENECT_DEPTH_MM_MAX_VALUE);
                Vec3b &pixel_color = depth_mat_tmp.at<Vec3b>(i,j);
                pixel_color[0] = color;
                pixel_color[1] = color;
                pixel_color[2] = color;
            } else {
                Vec3b &pixel_color = depth_mat_tmp.at<Vec3b>(i,j);
                pixel_color[0] = 0;
                pixel_color[1] = 0;
                pixel_color[2] = 0;
            }
        }
    cv::imshow("DEPTH", depth_mat_tmp);
}

// ---------------------------------------------------------
// Name:		show_rgb
// ---------------------------------------------------------
// Description:	show rgb frame as a video
// ---------------------------------------------------------

void show_rgb (uchar *rgb) {
    Mat img_bgr_mat;
    Mat img_rgb_mat = Mat(kinect,CV_8UC3,rgb);
    cvtColor(img_rgb_mat, img_bgr_mat, CV_RGB2BGR);
    imshow("RGB", img_bgr_mat);
}

// ---------------------------------------------------------
// Name:		clean_image
// ---------------------------------------------------------
// Description:	manually treshold a CV_16UC1 image, since
// the chances of opencv working for this are kind hit-or-miss
// between versions and operatins systems
// ---------------------------------------------------------

Mat clean_image(Mat input_image, int near, int far, int left, int right, int top, int bottom){
    Mat output_image;
    input_image.copyTo(output_image);
    for(int i = 0; i < output_image.size().height; ++i)
    {
        for(int j = 0; j < output_image.size().width; ++j)
        {
            uint16_t &pixel = output_image.at<uint16_t>(i, j);
            if(pixel > far || pixel < near || i < top || i > bottom || j < left || j > right){
                pixel = 0;
            }
        }
    }
    return output_image;
}

// ---------------------------------------------------------
// Name:		save_depth
// ---------------------------------------------------------
// Description:	create a file and save one depth frame
// ---------------------------------------------------------

int save_depth (uint16_t *depth) {
    Mat depth_mat = Mat(kinect,CV_16UC1, depth);
    std::ostringstream oss;
    oss << capture_name << "_depth_" << count_depth*capture_step << ".png";
    depth_mat = clean_image(depth_mat,depth_min,depth_max,left_plane,right_plane,top_plane,bottom_plane);
    imwrite(oss.str(), depth_mat);
    printf("%s saved!\n", oss.str().c_str());
    fflush(stdout);
    ++count_depth;
    return 1;
}

// ---------------------------------------------------------
// Name:		save_depth_burst
// ---------------------------------------------------------
// Description:	create a file and save one depth frame from
// a burst sequence
// ---------------------------------------------------------

int save_depth_burst(uint16_t *depth, unsigned int frame_count) {
    Mat depth_mat = Mat(kinect,CV_16UC1, depth);
    std::ostringstream oss;
    oss << capture_name << "_burst_" << count_burst*capture_step << "_" << frame_count << ".png";
    depth_mat = clean_image(depth_mat,depth_min,depth_max,left_plane,right_plane,top_plane,bottom_plane);
    imwrite(oss.str(), depth_mat);
    printf("%s saved!\n", oss.str().c_str());
    fflush(stdout);
    return 1;
}

// ---------------------------------------------------------
// Name:		save_rgb
// ---------------------------------------------------------
// Description:	create a file and save one rgb frame
// ---------------------------------------------------------

int save_rgb(uchar *rgb) {
    Mat img_rgb_mat = Mat(kinect,CV_8UC3,rgb);
    Mat img_bgr_mat;
    std::ostringstream oss;
    oss << capture_name << "_color_" << count_color*capture_step << ".png";
    cvtColor(img_rgb_mat, img_bgr_mat, CV_RGB2BGR);
    imwrite(oss.str(), img_bgr_mat);
    printf("%s saved!\n", oss.str().c_str());
    fflush(stdout);
    ++count_color;
    return 1;
}

// ---------------------------------------------------------
// Name:		save_ply
// ---------------------------------------------------------
// Description:	create a file and save a .ply point cloud
// with both depth and color information (now working!)
// ---------------------------------------------------------
int save_ply(Mat depth_mat, Mat color_mat, int count) {
    std::ostringstream oss;
    oss << capture_name << "_mesh_" << count*capture_step << ".ply";

    FILE *fp;

    if ((fp = fopen(oss.str().c_str(), "w")) == NULL) {
        printf("\nError: while creating file %s", oss.str().c_str());
        return 0;
    }

    //bilateral filtering of the depth image
    //disabled for now because super resolution
    //ends up smoothing out the noise
    /*depth_mat.convertTo(depth_mat_float, CV_32FC1);
    bilateralFilter(depth_mat_float, depth_mat_float_filtered, -1, 30, 4.5);
    printf("Bilateral Filter Done!\n");
    depth_mat_float_filtered.convertTo(depth_mat_filtered,CV_16UC1);
    depth_mat = Mat(depth_mat_filtered);*/

    //normals via cross-product
    Mat normal_mat = Mat(depth_mat.size(), CV_32FC3);
    for(int i = 0; i < depth_mat.size().height; ++i)
    {
        for(int j = 0; j < depth_mat.size().width; ++j)
        {
            uint16_t right = 0, left = 0, down = 0, up = 0;

            if(i > 0 && j > 0 && i < depth_mat.size().height-1 && j < depth_mat.size().width-1) {
                right = depth_mat.at<uint16_t>(i+1, j);
                left = depth_mat.at<uint16_t>(i-1, j);
                down = depth_mat.at<uint16_t>(i, j+1);
                up = depth_mat.at<uint16_t>(i, j-1);
            }

            //threshold (as per Hinterstoisser et al (2011))
            if(abs(depth_mat.at<uint16_t>(i, j) - right) > 20) {
                right = 0;
            }
            if(abs(depth_mat.at<uint16_t>(i, j) - left) > 20) {
                left = 0;
            }
            if(abs(depth_mat.at<uint16_t>(i, j) - down) > 20) {
                down = 0;
            }
            if(abs(depth_mat.at<uint16_t>(i, j) - up) > 20) {
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
    //iterate to count the number of vertices, needed for .ply header
    for (int i = 0; i < depth_mat.size().height; i++) {
        for (int j = 0; j < depth_mat.size().width; j++) {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i,j);

            if(z_in_mm != 0 && z_in_mm >= depth_min && z_in_mm <= depth_max && \
               i >= top_plane && i <= bottom_plane && j >= left_plane && j <= right_plane) {
                ++num_vertices;
            }
        }
    }

    //then a second time to write the geometry into the .ply file
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
    for (int i = 0; i < depth_mat.size().height; i++) {
        for (int j = 0; j < depth_mat.size().width; j++) {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i,j);
            double fx = 572.882768, fy = 542.739980, cx = 314.649173, cy = 240.160459;
            if(z_in_mm != 0 && z_in_mm >= depth_min && z_in_mm <= depth_max && \
                i >= top_plane && i <= bottom_plane && j >= left_plane && j <= right_plane) {
                double vx = z_in_mm * (j - cx) * (1/fx);
                double vy = -z_in_mm * (i - cy) * (1/fy);
                Vec3b color = color_mat.at<Vec3b>(i,j);
                Vec3f normal = normal_mat.at<Vec3f>(i,j);
                fprintf(fp, "%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf %d %d %d\n",
                        (double) vx, (double) vy, (double) -z_in_mm, (double) normal[0],
                        (double) normal[1], (double) normal[2], color[0], color[1], color[2]);
            }
        }
    }

    fclose(fp);
    fflush(fp);

    printf("%s saved!\n", oss.str().c_str());
    fflush(stdout);

    return 1;
}

int main(int argc, char **argv)
{
    if(argc > 1){
        capture_name = std::string(argv[1]);
    }
    if(argc > 2){
        capture_step = std::stoi(std::string(argv[2]));
    }

    std::cout << capture_name << " " << capture_step << std::endl;

    namedWindow("RGB", CV_WINDOW_AUTOSIZE);
    namedWindow("DEPTH", CV_WINDOW_AUTOSIZE);

    kinect.width = 640;
    kinect.height = 480;

    //Trackbar
    createTrackbar("Near Plane (mm)", "DEPTH", &depth_min, 4000, NULL);
    createTrackbar("Far Plane (mm)", "DEPTH", &depth_max, 4000, NULL);
    createTrackbar("Left Plane (px)", "DEPTH", &left_plane, kinect.width-1, NULL);
    createTrackbar("Right Plane (px)", "DEPTH", &right_plane, kinect.width-1, NULL);
    createTrackbar("Top Plane (px)", "DEPTH", &top_plane, kinect.height-1, NULL);
    createTrackbar("Bottom Plane (px)", "DEPTH", &bottom_plane, kinect.height-1, NULL);

    int key = 0;

    freenect_sync_set_tilt_degs(0,0);

    while(key = waitKey(1))
    {
        //std::cout << key << std::endl;
        //opencv apparently cant decide whether 'no key' is -1 or 255. (Yes, I tried masking and casting)
        if(key == 255 || key==-1) {
            uint32_t timestamp;
            uint32_t timestamp_depth;
            unsigned char *data;
            uint16_t *depth_data;
            #pragma omp parallel num_threads(2)
            {
                int thread_num = omp_get_thread_num();
                if(thread_num == 0) {
                    freenect_sync_get_video((void**)(&data), &timestamp, 0, FREENECT_VIDEO_RGB);
                    show_rgb(data);
                } else if(thread_num == 1) {
                    freenect_sync_get_depth((void**)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
                    show_depth(depth_data);
                }
            }
        } else {
            if (key == CLOSE) {
                break;
            }
            if (key == DEPTH) {
                uint32_t timestamp_depth;
                uint16_t *depth_data;
                freenect_sync_get_depth((void**)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
                save_depth(depth_data);
            }
            if (key == BURST) {
                uint32_t timestamp_depth;
                uint16_t *depth_data;
                for(unsigned int i = 0; i < SR_SIZE; ++i){
                    freenect_sync_get_depth((void**)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
                    save_depth_burst(depth_data, i);
                }
                ++count_burst;
            }
            if (key == COLOR) {
                uint32_t timestamp;
                unsigned char *data;
                freenect_sync_get_video((void**)(&data), &timestamp, 0, FREENECT_VIDEO_RGB);
                save_rgb(data);
            }
            if (key == DEPTH_MESH) {
                uint32_t timestamp_depth;
                uint16_t *depth_data;
                freenect_sync_get_depth((void**)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
                uint32_t timestamp_color;
                unsigned char *color_data;
                freenect_sync_get_video((void**)(&color_data), &timestamp_color, 0, FREENECT_VIDEO_RGB);
                save_ply(Mat(kinect,CV_16UC1,depth_data),Mat(kinect,CV_8UC3,color_data),count_depth_mesh);
                ++count_depth_mesh;
            }
            if (key == ALL) {
                //depth
                uint32_t timestamp_depth;
                uint16_t *depth_data;
                freenect_sync_get_depth((void**)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
                save_depth(depth_data);

                //color
                uint32_t timestamp;
                unsigned char *data;
                freenect_sync_get_video((void**)(&data), &timestamp, 0, FREENECT_VIDEO_RGB);
                save_rgb(data);

                //depth mesh
                freenect_sync_get_depth((void**)(&depth_data), &timestamp_depth, 0, FREENECT_DEPTH_REGISTERED);
                uint32_t timestamp_color;
                unsigned char *color_data;
                freenect_sync_get_video((void**)(&color_data), &timestamp_color, 0, FREENECT_VIDEO_RGB);
                save_ply(Mat(kinect,CV_16UC1,depth_data),Mat(kinect,CV_8UC3,color_data),count_depth_mesh);
                ++count_depth_mesh;

                //burst image
                uint32_t timestamp_burst;
                uint16_t *burst_data;
                for(unsigned int i = 0; i < SR_SIZE; ++i){
                    freenect_sync_get_depth((void**)(&burst_data), &timestamp_burst, 0, FREENECT_DEPTH_REGISTERED);
                    save_depth_burst(burst_data, i);
                }
                ++count_burst;
            }
        }
    }

    return 0;
}