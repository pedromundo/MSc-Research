#include "libfreenect.h"
#define ACCUM_SIZE 30
#define BILATERAL
#include <opencv2/opencv.hpp>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <pthread.h>
#include <sstream>

#define CLOSE 27
#define DEPTH 'q'
#define COLOR 'w'
#define MESH 'e'
#define SUPER 's'

using namespace cv;

pthread_t freenect_thread;

pthread_mutex_t mutex_capture = PTHREAD_MUTEX_INITIALIZER;

freenect_context *context;
freenect_device *device;
freenect_frame_mode depthFrame;
freenect_frame_mode rgbFrame;

static int freenect_angle = 0;

static int captureDEPTH  = 0;	// parameter that decides if current frame should be saved
static int captureRGB    = 0;   // parameter that decides if current frame should be saved
static int captureMESH   = 0;   // parameter that decides if current frames should be meshed
static int captureACCUM  = 0;   // parameter that decides if superres image should be saved

static int framesLeft = ACCUM_SIZE;     //When accumulating multiple frames, this is used as counter

static int countDEPTH = 0;		// to control DEPTH frame saving
static int countRGB   = 0;		// to control RGB frame saving
static int countMESH  = 0;		// to control Mesh frame saving
static int countACCUM = 0;      // to control SUPERRES frame saving

//Depth thresholds in mm
int depthMin = 300;
int depthMax = 800;

static Size kinect;
static Mat imgRGB;		// for RGB image video
static Mat imgBGR;		// for RGB image video after converting
static Mat depthMat;    //raw depth data
static Mat imgDEPTH;    //converted/normalized depth image
//bilinear filtering
static Mat depthMatFiltered;      //filtered depth data
static Mat depthMatFloat;         //depth data converted to float
static Mat depthMatFloatFiltered; //filtered depth data converted to float
//frame accumulation
static Mat accumulatedDepths[ACCUM_SIZE]; //depth frames for accumulation
static Mat accumulatedDepthMat;   //final accumulated depth frame

/********************************************************************************************************** 

########     ################    
# main # --- # start_kinect # 
########     ################
                    |
             #######################     #################     ##############	  ########
             # freenect_threadfunc # --- # depthCallback # --- # show_depth # --- # lerp #
             #######################     #################     ##############	  ########
                    |                           |
                    |                           |              ##############     #############
                    |                           -------------- # save_depth # --- # file_name #
                    |                                          ##############     #############
                    |
                    |                ###############     ############
                    ---------------- # rgbCallback # --- # show_rgb #
                                     ###############     ############
                                         |
                                         |              ############       #############
                                         -------------- # save_ply # ----- # file_name #
                                         |				############       #############
                                         |
                                         |              ############       #############
                                         -------------- # save_rgb # ----- # file_name #
                                                        ############       #############

**********************************************************************************************************/

inline double lerp(double a, double b, double f) 
{
    return (a * (1.0 - f)) + (b * f);
}

// ---------------------------------------------------------
// Name:		file_name
// ---------------------------------------------------------
// Description:	adapt the name of the file with each frame 
// ---------------------------------------------------------

void file_name(char *name, int number, char *type) {
    char n[10];
    sprintf(n, "%d", number);
    strcat(name, n);
    strcat(name, ".");
    strcat(name, type);
}

// ---------------------------------------------------------
// Name:		show_depth
// ---------------------------------------------------------
// Description:	transform a depth frame information into color
// 				and show as a video 
// ---------------------------------------------------------

void show_depth (uint16_t *depth) {	
    int i, j;

    for ( i = 0; i < kinect.height; i++)
        for ( j = 0; j < kinect.width; j++) {
            uint16_t depth_in_mm = depthMat.at<uint16_t>(i,j);
            if (depth_in_mm != 0 && depth_in_mm >= depthMin && depth_in_mm <= depthMax) {
                uint16_t color = (uint16_t) lerp(255.0,0.0,(double) depth_in_mm/(depthMax));
                Vec3b &pixelColor = imgDEPTH.at<Vec3b>(i,j);
                pixelColor[0] = color;
                pixelColor[1] = color;
                pixelColor[2] = color;
            } else {
                Vec3b &pixelColor = imgDEPTH.at<Vec3b>(i,j);
                pixelColor[0] = 0;
                pixelColor[1] = 0;
                pixelColor[2] = 0;
            }
        }
    imshow("DEPTH", imgDEPTH);
}


// ---------------------------------------------------------
// Name:		show_rgb
// ---------------------------------------------------------
// Description:	show rgb frame as a video
// ---------------------------------------------------------

void show_rgb (uchar *rgb) {	
    imgRGB = Mat(kinect,CV_8UC3,rgb);
    cvtColor(imgRGB, imgBGR, CV_RGB2BGR);
    imshow("RGB", imgBGR);
}


// ---------------------------------------------------------
// Name:		save_depth
// ---------------------------------------------------------
// Description:	create a file and save one depth frame 
// ---------------------------------------------------------

int save_depth (){
    char name[40];
    char ext[4] = {'b','m','p','\0'};
    strcpy(name, "depth_image");
    file_name(name, countDEPTH,ext);

    imwrite(name, imgDEPTH);

    ++countDEPTH;
    return 1;
}


// ---------------------------------------------------------
// Name:		save_rgb
// ---------------------------------------------------------
// Description:	create a file and save one rgb frame 
// ---------------------------------------------------------

int save_rgb() {   
    char name[40];
    char ext[4] = {'b','m','p','\0'};
    strcpy(name, "color_image");

    file_name(name, countRGB,ext);

    imwrite(name,imgBGR);
    ++countDEPTH;

    return 1;
}

// ---------------------------------------------------------
// Name:		save_ply
// ---------------------------------------------------------
// Description:	create a file and save a xyz polygonal mesh
// with both depth and color information (still not working) 
// ---------------------------------------------------------

int save_ply(Mat depthMat) {
    char name[40];
    char ply[4] = {'p','l','y','\0'};

    strcpy(name, "xyzrgb");
    file_name(name,countMESH,ply);

    FILE *fp;

    if ((fp = fopen(name, "w")) == NULL) {
        printf("\nError: while creating file %s", name);
        return 0;
    }

    int i, j;

    uint32_t num_vertices = 0;

    std::ostringstream oss;
    oss << "depthRaw_" << countMESH << ".png";
    imwrite(oss.str(),depthMat);

    //iterate to count the number of vertices
    for (i = 0; i < kinect.height; i++){
        for (j = 0; j < kinect.width; j++) {
            uint16_t z_in_mm = depthMat.at<uint16_t>(i,j);
            if(z_in_mm != 0 && z_in_mm >= depthMin && z_in_mm <= depthMax){
                ++num_vertices;
            }
        }
    }

    depthMat.convertTo(depthMatFloat, CV_32FC1);
    bilateralFilter(depthMatFloat, depthMatFloatFiltered, 36, 100, 100);
    printf("Bilateral Filter Done!\n");
    depthMatFloatFiltered.convertTo(depthMatFiltered,CV_16UC1);
    depthMat = Mat(depthMatFiltered);

    //then a second time to write the .ply file
    fprintf(fp,"ply\n");
    fprintf(fp,"format ascii 1.0\n");
    fprintf(fp,"element vertex %d\n",num_vertices);
    fprintf(fp,"property float x\n");
    fprintf(fp,"property float y\n");
    fprintf(fp,"property float z\n");
    fprintf(fp,"property uchar red\n");
    fprintf(fp,"property uchar green\n");
    fprintf(fp,"property uchar blue\n");
    //fprintf(fp,"element face 0\n");
    //fprintf(fp,"property list uchar int vertex_indices\n");
    fprintf(fp,"end_header\n");
    for (i = 0; i < kinect.height; i++){
        for (j = 0; j < kinect.width; j++) {
            uint16_t z_in_mm = depthMat.at<uint16_t>(i,j);
            double cx = 313.68782938, cy = 259.01834898, fx_inv = 1 / 526.37013657, fy_inv = fx_inv;
            if(z_in_mm != 0 && z_in_mm >= depthMin && z_in_mm <= depthMax){
                double vx = z_in_mm * (j - cx) * fx_inv;
                double vy = -z_in_mm * (i - cy) * fy_inv;
                fprintf(fp, "%.6lf ", (double)vx);
                fprintf(fp, "%.6lf ", (double)vy);
                fprintf(fp, "%.6lf ", (double)-z_in_mm);

                Vec3b rgbColor = imgRGB.at<Vec3b>(i,j);
                fprintf(fp, "%d ", rgbColor[0]);
                fprintf(fp, "%d ", rgbColor[1]);
                fprintf(fp, "%d\n", rgbColor[2]);
            }
        }
    }

    fclose(fp);
    fflush(fp);

    printf(".ply mesh %d saved!\n", countMESH++);
    fflush(stdout);

    return 1;
}


// ---------------------------------------------------------
// Name:		depthCallback
// ---------------------------------------------------------
// Description:	callback that receives each depth frame.
// ---------------------------------------------------------

void depthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {    
    depthMat = Mat(kinect,CV_16UC1, depth);
    if (captureMESH){
        captureMESH = 0;
        pthread_mutex_lock(&mutex_capture);		//LOCK
        save_ply(depthMat);
        pthread_mutex_unlock(&mutex_capture);	//UNLOCK
    }

    if (captureACCUM && framesLeft >= 0){
        Mat(kinect,CV_16UC1,depth).copyTo(accumulatedDepths[ACCUM_SIZE-framesLeft--]);
        if(framesLeft < 1){
            Mat final;
            accumulatedDepths[0].copyTo(final);
            for(int i=1;i<ACCUM_SIZE;++i){
                Mat mask;
                final.convertTo(mask, CV_8UC1);
                threshold(mask, mask, 0.1, UINT8_MAX, THRESH_BINARY_INV);
                add(final, accumulatedDepths[i], final, mask);
                imshow("Final", final * 32);
                waitKey(30);
            }
            accumulatedDepthMat = final;
            save_ply(accumulatedDepthMat);
            captureACCUM = 0;
            framesLeft = ACCUM_SIZE;
            ++countACCUM;
        }
    }

    if (!captureDEPTH)
        show_depth((uint16_t*)depth);
    else {
        captureDEPTH = 0;
        pthread_mutex_lock(&mutex_capture);		//LOCK
        save_depth();
        pthread_mutex_unlock(&mutex_capture);	//UNLOCK
    }
}


// ---------------------------------------------------------
// Name:		rgbCallback
// ---------------------------------------------------------
// Description:	callback that receives each rgb frame.
// ---------------------------------------------------------

void rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp) {
    if (!captureRGB)
        show_rgb((uchar*)rgb);
    else {
        captureRGB = 0;
        pthread_mutex_lock(&mutex_capture);		//LOCK
        save_rgb();
        pthread_mutex_unlock(&mutex_capture);	//UNLOCK
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
    freenect_set_tilt_degs(device,freenect_angle);

    // SET STATE OF THE LED
    freenect_set_led(device,LED_RED);

    // SET CALLBACK FOR DEPTH AND VIDEO INFORMATION RECEIVED EVENT
    freenect_set_depth_callback(device, depthCallback);
    freenect_set_video_callback(device, rgbCallback);

    // SET CURRENT MODE FOR DEPTH AND VIDEO. MODE CAN'T BE CHANGED WHILE STREAMING IS ACTIVE
    depthFrame = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED);
    rgbFrame = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);

    freenect_set_depth_mode(device, depthFrame);
    freenect_set_video_mode(device, rgbFrame);

    // START DEPTH AND VIDEO STREAM
    freenect_start_depth(device);
    freenect_start_video(device);

    int key = 0;

    while((key = (uchar)cvWaitKey(1)) && freenect_process_events(context) >= 0) {
        if (key == CLOSE)
            break;
        if (key == DEPTH) {
            captureDEPTH = 1;
        }
        if (key == COLOR) {
            captureRGB = 1;
        }
        if (key == MESH) {
            captureMESH = 1;
        }
        if (key == SUPER) {
            captureACCUM = 1;
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
    //freenect_set_log_level(context, FREENECT_LOG_DEBUG);
    freenect_set_log_level(context, FREENECT_LOG_ERROR);

    // SET SUBDEVICES TO BE CALLED
    freenect_select_subdevices(context, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    int num_devices = freenect_num_devices (context);
    printf ("Number of devices found: %d\n", num_devices);

    int device_number = 0;

    // CHECK DEVICES
    if (argc > 1)
        device_number = atoi(argv[1]);

    if (num_devices < 1)
        return 1;

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
    createTrackbar("Min Depth.", "DEPTH", &depthMin, FREENECT_DEPTH_MM_MAX_VALUE, NULL);
    createTrackbar("Max Depth.", "DEPTH", &depthMax, FREENECT_DEPTH_MM_MAX_VALUE, NULL);

    kinect.width = 640;
    kinect.height = 480;

    imgRGB = Mat(kinect, CV_8UC3);
    imgBGR = Mat(kinect, CV_8UC3);
    imgDEPTH = Mat(kinect, CV_8UC3);
    depthMat = Mat(kinect, CV_16UC1);

    // INITIALIZE KINECT DEVICE
    int problem = start_kinect(argc, argv);
    if (problem) exit(1);

    // SUSPEND EXECUTION OF CALLING THREAD
    pthread_join(freenect_thread, NULL);

    return 0;
}
