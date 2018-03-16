#include <opencv2/opencv.hpp>
#include <sstream>
using namespace cv;

int main(int argc, char **argv){
    Mat lr_images[16];
    Mat alignment_matrices[16];    
    for(size_t i = 0; i < 16;++i){
        std::ostringstream oss_in;
        oss_in << "cap_depth_" << i << ".png";
        lr_images[i] = imread(oss_in.str(),CV_LOAD_IMAGE_ANYDEPTH);
        lr_images[i].convertTo(lr_images[i],CV_32FC1);
        //ecc subpixel alignment
        findTransformECC(lr_images[0],lr_images[i],alignment_matrices[i], MOTION_AFFINE, \
        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 400, 1E-12));
        std::cout << "Alignment " << i << "-> 0 = " << alignment_matrices[i] << std::endl;
        if(i > 0){
            //warp all to template image
            warpAffine(lr_images[i],lr_images[i],alignment_matrices[i],lr_images[0].size(),CV_WARP_INVERSE_MAP);
            lr_images[i].convertTo(lr_images[i],CV_16UC1);
            std::ostringstream oss_out;
            oss_out << "cap_depth_" << i << "_registered.png";
            lr_images[i] = lr_images[i] * 20;
            //upsample - not upscale
            pyrUp(lr_images[i],lr_images[i],lr_images[i].size()*2);            
            imwrite(oss_out.str(),lr_images[i]);
        }else{
            Mat tmp;
            lr_images[i].convertTo(lr_images[i],CV_16UC1);
            std::ostringstream oss_out;
            oss_out << "cap_depth_" << i << "_registered.png";
            tmp = lr_images[i] * 20;
            //upsample - not upscale
            pyrUp(tmp,tmp,tmp.size()*2);
            imwrite(oss_out.str(),tmp);
            //back to 32F to keep doing ECC
            lr_images[i].convertTo(lr_images[i],CV_32FC1);
        }
    }

    Mat hr_image = Mat(lr_images[0].size()*2,CV_16UC1);
}