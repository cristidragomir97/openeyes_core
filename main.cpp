#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include "serial.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#define MAX_DISTANCE 8000
#define WIDTH 640
#define HEIGHT 480

int vibrators = 32;
int slices = 10;
int bins = 96;

int h_step = 0;
int w_step = 0;
int d_step = 0;

using namespace cv;

cv::Scalar get_colour(double v, double vmin, double vmax){
    double r, g, b = 0;
    double dv;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    if (v < (vmin + 0.5 * dv)) {
        r = 2 * (v - vmin) / dv;
        g = 2 * (v - vmin) / dv;
        b = 1;
    }
    else if (v >= (vmin + 0.5 * dv)) {
        b = 1 + 2 * (vmin + 0.5 * dv - v) / dv;
        g = 1 + 2 * (vmin + 0.5 * dv - v) / dv;
        r = 1;
    }

    return cv::Scalar(b * 255, g * 255, r * 255);
}


void process_frame(rs2::depth_frame frame, cv::Mat image, cv::Mat out_image) {
    rs2::decimation_filter dec;
    rs2::spatial_filter spat;

    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);

    frame.apply_filter(dec).apply_filter(spat);
    h_step = HEIGHT / slices;
    w_step = WIDTH / vibrators;
    d_step = MAX_DISTANCE / bins;

    int l1_out[slices][vibrators];

    for (int w = 0; w < vibrators; w++) {
        for (int h = 0; h < slices; h++) {
            long p_sum = 1;
            long v_sum = 0;

            for (int j = h * h_step; j < (h + 1) * h_step; j+=1) {
                for (int i = w * w_step; i < (w + 1) * w_step; i+=1) {

                    long this_val = long(frame.get_distance(i, j) * 1000);
                    if(i > 20 && j > 20){
                        for (int k = 0; k < bins; k++){
                            if (this_val > d_step * k && this_val < d_step * (k + 1)){
                                v_sum += this_val;
                                p_sum += 1;
                            }
                        }
                    }
                }
            }


            long avg = v_sum / p_sum;
           // l1_out[w][h] = avg;
            char buffer[5];
            sprintf(buffer,"%d", avg);
            cv::rectangle(out_image, cv::Point((w+1) * w_step,(h+1) * h_step),  cv::Point(w * w_step, h * h_step), get_colour(avg, 0, 5000), -1);
        }
    }

    for (int w = 0; w < vibrators; w++) {
        for (int h = 0; h < slices; h++) {
            std::cout <<  l1_out[w][h]  << " " ;
        }
        std::cout << "\n";
    }

    std::cout << "___________________________" << "\n";
}


int main(int argc, char * argv[]) try
{

    //serial port = serial(PORT, method);
    //pport = &port;
    //pport -> update(0, 4096, 100);

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer color_map;

    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 15);
    pipe.start(cfg);


    const auto window_name = "openeyez_core_v0";
    const auto out_window = "openeyes_core out";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    namedWindow(out_window, WINDOW_AUTOSIZE);


    createTrackbar("vibrators", window_name, &vibrators, 50);
    createTrackbar("slices", window_name, &slices, 50);
    createTrackbar("bins", window_name, &bins, 300);




    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame();
        rs2::frame colorized = depth.apply_filter(color_map);


        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)colorized.get_data(), Mat::AUTO_STEP);
        Mat img(Size(w, h), CV_8UC3,Scalar(0,0,0));


        process_frame(depth, image, img);

        for(int i=0;i<h;i+=h_step)
            cv::line(image, Point(0, i), Point(w, i), cv::Scalar(0, 255, 255));

        for(int i=0;i<w;i+=w_step)
            cv::line(image, Point(i, 0), Point(i, h), cv::Scalar(255, 0, 255));


        // Update the window with new data
        imshow(window_name, image);
        imshow(out_window, img);

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}