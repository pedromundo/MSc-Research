#include <libfreenect/libfreenect.h>

#include <opencv2/opencv.hpp>
#include <sstream>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <pthread.h>

//Escape closes the applications
#define CLOSE 27
//Image capture keys
#define DEPTH 'q'
#define ACCUM 'w'
#define COLOR 'e'
#define ALL 'A'
//Mesh capture keys
#define DEPTH_MESH 'Q'
#define ACCUM_MESH 'W'

//Constants
#define MESH_TYPE_REGULAR 1
#define MESH_TYPE_ACCUMULATED 2
#define ACCUM_SIZE 6

using namespace cv;

pthread_t freenect_thread;

pthread_mutex_t mutex_capture = PTHREAD_MUTEX_INITIALIZER;

freenect_context *context;
freenect_device *device;
freenect_frame_mode depth_frame;
freenect_frame_mode rgb_frame;

static int freenect_angle = 0;

//Control Variables
static int capture_depth = 0;
static int capture_color = 0;
static int capture_accum = 0;
static int capture_depth_mesh = 0;
static int capture_accum_mesh = 0;

static int count_depth = 0;
static int count_color = 0;
static int count_accum = 0;
static int count_depth_mesh = 0;
static int count_accum_mesh = 0;

static std::string capture_name = "cap";
static int capture_step = 30;

static int frames_left = ACCUM_SIZE;
static Mat accumulated_depths[ACCUM_SIZE];


//Depth thresholds in mm
int depth_min = 650;
int depth_max = 1300;

//OpenCV variables
static Size kinect;
static Mat img_rgb_mat;    // for RGB image video
static Mat depth_mat;    //raw depth data
static Mat normal_mat;    //raw normal data

//TODO: I dont need all of these since conversion works in-place
static Mat depth_mat_filtered;      //filtered depth data
static Mat depth_mat_float;         //depth data converted to float
static Mat depth_mat_float_filtered; //filtered depth data converted to float

/**********************************************************************************************************

########     ################
# main # --- # start_kinect #
########     ################
                    |
             #######################     #################     ##############	  ########
             # freenect_threadfunc # --- # depthCallback # --- # show_depth # --- # lerp #
             #######################     #################     ##############	  ########
                    |                           |
                    |                           |              ##############
                    |                           -------------- # save_depth #
                    |                                          ##############
                    |
                    |                ###############    ############
                    ---------------- # rgbCallback # -- # show_rgb #
                                     ###############    ############
                                         |
                                         |              ############
                                         -------------- # save_ply #
                                         |				############
                                         |
                                         |              ############
                                         -------------- # save_rgb #
                                                        ############

**********************************************************************************************************/

inline uint16_t vec3b_to_uint16(Vec3b value) {
    uint16_t res = 0;
    res = res | value[0];
    res = res << 8;
    res = res | value[1];
    return res;
}

inline Vec3b uint16_to_vec3b(uint16_t value) {
    Vec3b res;
    res[0] = (unsigned char)((value & 0xFF00) >> 8);
    res[1] = (unsigned char)(value & 0x00FF);
    res[2] = 0;
    return res;
}

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
    Mat depth_mat_tmp = Mat(kinect, CV_8UC3);
    int i, j;
    for ( i = 0; i < kinect.height; i++)
        for ( j = 0; j < kinect.width; j++) {
            uint16_t depth_in_mm = depth_mat.at<uint16_t>(i,j);
            if (depth_in_mm != 0 && depth_in_mm >= depth_min && depth_in_mm <= depth_max) {
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
    img_rgb_mat = Mat(kinect,CV_8UC3,rgb);
    cvtColor(img_rgb_mat, img_bgr_mat, CV_RGB2BGR);
    imshow("RGB", img_bgr_mat);
}

// ---------------------------------------------------------
// Name:		save_depth
// ---------------------------------------------------------
// Description:	create a file and save one depth frame
// ---------------------------------------------------------

int save_depth () {
    std::ostringstream oss;
    oss << capture_name << "_depth_" << count_depth*capture_step << ".png";
    Mat clean_mat(depth_mat);
    threshold(clean_mat,clean_mat,depth_max,0,cv::ThresholdTypes::THRESH_TOZERO_INV);
    threshold(clean_mat,clean_mat,depth_min,0,cv::ThresholdTypes::THRESH_TOZERO);
    imwrite(oss.str(), clean_mat);
    printf("%s saved!\n", oss.str().c_str());
    fflush(stdout);
    ++count_depth;
    return 1;
}

// ---------------------------------------------------------
// Name:		save_rgb
// ---------------------------------------------------------
// Description:	create a file and save one rgb frame
// ---------------------------------------------------------

int save_rgb() {
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
// Description:	create a file and save a .ply polygonal mesh
// with both depth and color information (now working!)
// ---------------------------------------------------------
int save_ply(Mat depth_mat, int mesh_type, int count, float min_value=std::numeric_limits<float>::min(),\
                 float max_value=std::numeric_limits<float>::max()) {
    std::ostringstream oss;
    if(mesh_type == MESH_TYPE_REGULAR) {
        oss << capture_name << "_mesh_" << count*capture_step << ".ply";
    } else if(mesh_type == MESH_TYPE_ACCUMULATED) {
        oss << capture_name << "_accummesh_" << count*capture_step << ".ply";
    }
    FILE *fp;

    if ((fp = fopen(oss.str().c_str(), "w")) == NULL) {
        printf("\nError: while creating file %s", oss.str());
        return 0;
    }

    int i, j;

    //bilateral filtering of the depth image
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
    for (i = 0; i < depth_mat.size().height; i++) {
        for (j = 0; j < depth_mat.size().width; j++) {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i,j);

            if(z_in_mm != 0 && z_in_mm >= min_value && z_in_mm <= max_value) {
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
    for (i = 0; i < depth_mat.size().height; i++) {
        for (j = 0; j < depth_mat.size().width; j++) {
            uint16_t z_in_mm = depth_mat.at<uint16_t>(i,j);
            double cx = 313.68782938, cy = 259.01834898, fx_inv = 1 / 526.37013657, fy_inv = fx_inv;

            if(z_in_mm != 0 && z_in_mm >= min_value && z_in_mm <= max_value) {
                double vx = z_in_mm * (j - cx) * fx_inv;
                double vy = -z_in_mm * (i - cy) * fy_inv;
                //this is going to be a bit misaligned, not that bad if the
                //'centered' image is captured last
                Vec3b color = img_rgb_mat.at<Vec3b>(i,j);
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

Mat superresolve(Mat lr_images[], unsigned int image_count, int resample_factor) {
    Mat lr_images_upsampled[image_count]; //stored as float
    Mat alignment_matrices[image_count]; //affine transform matrices
    Mat hr_image;

    //Registration Phase - Subpixel registration of all LR images to the first
    #pragma omp parallel for
    for(size_t i = 0; i < image_count; ++i) {
        Mat template_image = lr_images[0]; //first LR image
        findTransformECC(template_image,lr_images[i],alignment_matrices[i], MOTION_AFFINE, \
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 200, 1E-12));
        std::cout << "Alignment " << i << "-> 0 = " << alignment_matrices[i] << std::endl;
    }

    //Upsample + Warp Phase - Upsample all LR images and use registration information
    //technically the resize operation should happen before the warpAffine, but this
    //is not working in this version of the program, whilst it worked on an older
    //version, ignoring this for now since the results are good
    for(size_t i = 0; i < image_count; ++i) {
        //Warp - Simplified to rigid transform because we aim to have as little
        //translation and rotation between the LR images as possible while still
        //modifying the intrinsics enough to have complementary data
        if(i > 0) {
            warpAffine(lr_images[i],lr_images[i],alignment_matrices[i], lr_images[i].size(),CV_WARP_INVERSE_MAP);
        }

        //Upsample - can use pyramids or perform a simple scale operation
        //results were exactly the same, still gotta find out why
        cv::resize(lr_images[i],lr_images_upsampled[i],lr_images[i].size()*resample_factor,0,0,INTER_NEAREST);
    }

    //Reconstruction Phase - For now, simply averaging the images, this reduces the
    //impact of the additive noise from the kinect sensor on the HR image (Richardt).
    //Nonlinear noise generated during the other phases will be removed using the
    //previously established limits of the depth info; i.e., we can not generate or
    //reconstruct information outside of the observed volume. This implementation
    //performs poorly in case the images are not very well aligned. In fact, even
    //changing the ECC epsilon from 1E-12 to 1E-06 adversely affected the results
    hr_image = Mat(lr_images_upsampled[0].size(),CV_32FC1);
    for(size_t i = 0; i < image_count; ++i) {
        addWeighted(hr_image,1.0,lr_images_upsampled[i],(1.0/image_count),0,hr_image);
    }

    //Downsample the HR image back to 640x480 to keep it valid within the coordinate
    //system of the Microsoft Kinect sensor
    cv::resize(hr_image,hr_image,hr_image.size()/resample_factor,0,0,INTER_NEAREST);
    hr_image.convertTo(hr_image,CV_16UC1);
    return hr_image;
}

// ---------------------------------------------------------
// Name:		depthCallback
// ---------------------------------------------------------
// Description:	callback that receives each depth frame.
// ---------------------------------------------------------

void depthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
    depth_mat = Mat(kinect,CV_16UC1, depth);
    //Mesh Capture
    if (capture_depth_mesh) {
        pthread_mutex_lock(&mutex_capture);
        threshold(depth_mat,depth_mat,depth_max,0,cv::ThresholdTypes::THRESH_TOZERO_INV);
        threshold(depth_mat,depth_mat,depth_min,0,cv::ThresholdTypes::THRESH_TOZERO);
        save_ply(depth_mat, MESH_TYPE_REGULAR, count_depth_mesh);
        capture_depth_mesh = 0;
        ++count_depth_mesh;
        pthread_mutex_unlock(&mutex_capture);
    }

    if ((capture_accum_mesh || capture_accum) && frames_left >= 0) {
        pthread_mutex_lock(&mutex_capture);
        Mat(kinect,CV_16UC1,depth).copyTo(accumulated_depths[ACCUM_SIZE-frames_left--]);
        if(frames_left < 1) {
            float global_min = std::numeric_limits<float>::max(), global_max = std::numeric_limits<float>::min();
            //Pre-processing: get minimum and maximum values from the LR images we use
            //this information to remove extreme values from the HR image because some
            //non-linear noise might be added on the resampling and fusion phases
            //(i.e. remove all values outside of the observed range)
            for(size_t i = 0; i < ACCUM_SIZE; ++i) {
                threshold(accumulated_depths[i],accumulated_depths[i],depth_max,0,cv::ThresholdTypes::THRESH_TOZERO_INV);
                threshold(accumulated_depths[i],accumulated_depths[i],depth_min,0,cv::ThresholdTypes::THRESH_TOZERO);
                accumulated_depths[i].convertTo(accumulated_depths[i],CV_32FC1);

                double local_min, local_max;
                Mat mask = accumulated_depths[i] > 0;

                minMaxLoc(accumulated_depths[i],&local_min,&local_max,NULL,NULL,mask);

                global_min = local_min<global_min?local_min:global_min;
                global_max = local_max>global_max?local_max:global_max;
            }

            printf("Observed Global Min: %.0lf\nObserved Global Max: %.0lf\n",global_min,global_max);

            Mat final_image = superresolve(accumulated_depths,ACCUM_SIZE,2);
            if(capture_accum_mesh) {
                save_ply(final_image, MESH_TYPE_ACCUMULATED, count_accum_mesh, global_min, global_max);
                capture_accum_mesh = 0;
                ++count_accum_mesh;
            }
            if(capture_accum) {
                std::ostringstream oss;
                oss << capture_name << "_accumdepth_" << count_accum*capture_step << ".png";
                imwrite(oss.str(),final_image);
                printf("%s saved!\n", oss.str().c_str());
                fflush(stdout);
                capture_accum = 0;
                ++count_accum;
            }
            frames_left = ACCUM_SIZE;
        }
        pthread_mutex_unlock(&mutex_capture);
    }

    //Image Capture/Display
    if (!capture_depth) {
        show_depth((uint16_t*)depth);
    } else {
        capture_depth = 0;
        pthread_mutex_lock(&mutex_capture);
        save_depth();
        pthread_mutex_unlock(&mutex_capture);
    }
}


// ---------------------------------------------------------
// Name:		rgbCallback
// ---------------------------------------------------------
// Description:	callback that receives each rgb frame.
// ---------------------------------------------------------

void rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp) {
    if (!capture_color)
        show_rgb((uchar*)rgb);
    else {
        capture_color = 0;
        pthread_mutex_lock(&mutex_capture);
        save_rgb();
        pthread_mutex_unlock(&mutex_capture);
    }
}

// ---------------------------------------------------------
// Name:		freenect_threadfunc
// ---------------------------------------------------------
// Description:	kinect's main function. Start the depth and
//				rgb capture.
// ---------------------------------------------------------

void *freenect_threadfunc(void *arg) {
    // SET THE TILT STATE OF DEVICE
    freenect_set_tilt_degs(device, freenect_angle);

    // SET STATE OF THE LED
    freenect_set_led(device,LED_BLINK_RED_YELLOW);

    freenect_set_flag(device, FREENECT_RAW_COLOR, FREENECT_ON);

    // SET CALLBACK FOR DEPTH AND VIDEO INFORMATION RECEIVED EVENT
    freenect_set_depth_callback(device, depthCallback);
    freenect_set_video_callback(device, rgbCallback);

    // SET CURRENT MODE FOR DEPTH AND VIDEO. MODE CAN'T BE CHANGED WHILE STREAMING IS ACTIVE
    depth_frame = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED);
    rgb_frame = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);

    freenect_set_depth_mode(device, depth_frame);
    freenect_set_video_mode(device, rgb_frame);

    // START DEPTH AND VIDEO STREAM
    freenect_start_depth(device);
    freenect_start_video(device);

    freenect_set_flag(device, FREENECT_RAW_COLOR, FREENECT_ON);
    freenect_set_flag(device, FREENECT_AUTO_EXPOSURE, FREENECT_ON);
    freenect_set_flag(device, FREENECT_AUTO_WHITE_BALANCE, FREENECT_ON);

    int key = 0;

    while((key = (uchar)waitKey(1)) && freenect_process_events(context) >= 0) {
        if (key == CLOSE)
            break;
        if (key == DEPTH) {
            capture_depth = 1;
        }
        if (key == COLOR) {
            capture_color = 1;
        }
        if (key == ACCUM) {
            capture_accum = 1;
        }
        if (key == DEPTH_MESH) {
            capture_depth_mesh = 1;
        }
        if (key == ACCUM_MESH) {
            capture_accum_mesh = 1;
        }
        if (key == ALL) {
            capture_depth = 1;
            capture_color = 1;
            capture_accum = 1;
            capture_depth_mesh = 1;
            capture_accum_mesh = 1;
        }
    }

    printf("\nShutting down streams...");

    freenect_set_led(device,LED_BLINK_GREEN);

    // STOP STREAM
    freenect_stop_depth(device);
    freenect_stop_video(device);

    // STOP DEVICE
    freenect_close_device(device);
    freenect_shutdown(context);

    printf("DONE!\n");

    return NULL;
}


// ---------------------------------------------------------
// Name:		start_kinect
// ---------------------------------------------------------
// Description:	start the device and create freenect thread
// ---------------------------------------------------------

int start_kinect (int argc, char **argv) {
    printf("\nLoading Kinect Camera...");

    // START FREENECT CONTEXT
    if (freenect_init(&context, NULL) < 0) {
        printf("ERROR: failed to start freenect_init()\n");
        return 1;
    }
    else {
        printf("COMPLETE!\n");
    }

    // SET MESSAGE LOGGING
    freenect_set_log_level(context, FREENECT_LOG_INFO);

    // SET SUBDEVICES TO BE CALLED
    freenect_select_subdevices(context, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    int num_devices = freenect_num_devices (context);
    printf ("Number of devices found: %d\n", num_devices);

    int device_number = 0;

    // Check Arguments
    if (argc > 1) {
        device_number = atoi(argv[1]);
    }

    if (argc > 2) {
        capture_name = std::string(argv[2]);
    }

    if (argc > 3) {
        capture_step = atoi(argv[3]);
    }

    printf("Capture Parameters: Name: %s; Step: %d\n", capture_name.c_str(), capture_step);
    fflush(stdout);

    if (num_devices < 1) {
        return 1;
    }

    // CALL DEVICE
    if (freenect_open_device(context, &device, device_number) < 0) {
        printf("ERROR: could not open device\n");
        return 1;
    }

    // CREATE PTHREAD AND START KINECT
    int start = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    if (start) {
        printf("ERROR: failed to create Kinect Thread\n");
        return 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    namedWindow("RGB", CV_WINDOW_AUTOSIZE);
    namedWindow("DEPTH", CV_WINDOW_AUTOSIZE);

    //Trackbar
    createTrackbar("Min Depth.", "DEPTH", &depth_min, 4000, NULL);
    createTrackbar("Max Depth.", "DEPTH", &depth_max, 4000, NULL);

    kinect.width = 640;
    kinect.height = 480;

    img_rgb_mat = Mat(kinect, CV_8UC3);
    depth_mat = Mat(kinect, CV_16UC1);

    // INITIALIZE KINECT DEVICE
    int problem = start_kinect(argc, argv);
    if (problem) exit(1);

    // SUSPEND EXECUTION OF CALLING THREAD
    pthread_join(freenect_thread, NULL);

    return 0;
}