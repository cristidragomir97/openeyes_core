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


#define PORT "/dev/ttyUSB0"
#define MAX_DISTANCE 10000
#define BASE_RATE 320
#define WIDTH 640
#define HEIGHT 480

using namespace cv;
using namespace rs2;
using namespace std;

Scalar get_colour(double v, double vmin, double vmax)
{
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

void average_depth(rs2::depth_frame frame, Mat tmp, bool display) {
    rs2::decimation_filter dec;
    rs2::spatial_filter spat;

    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, thereshold);

    frame.apply_filter(dec).apply_filter(spat);


    for (int w = 0; w < vibrators; w++) {
        for (int h = 0; h < slices; h++) {

            long p_sum = 1;
            long v_sum = 0;

            char buffer[5];

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

            if(display){
                sprintf(buffer,"%d",avg);
                rectangle(tmp, cv::Point((w+1) * w_step,(h+1) * h_step),  cv::Point(w * w_step, h * h_step), get_colour(avg, 0, MAX_DISTANCE), -1);
            }



          /// map_values(w, height_index, min_distance);
        }
    }

}

void init_rs(pipeline * p, config * c){
    c -> enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 90);
    p -> start(c);
}

auto init_gui(int * vibrators, int * slices, int  * bins){
    const auto w = "openeyes";
    namedWindow(w, WINDOW_AUTOSIZE);
    cv::createTrackbar("vibrators", w, vibrators, 50);
    cv::createTrackbar("slices", w, slices, 50);
    cv::createTrackbar("bins", w, bins, 2000);

    return w;
}

int main(int argc, char * argv[]) try {

    bool display = true;
    bool serial = true;

    int vibrators = 32;
    int slices = 32;
    int bins = 96;

    pipeline pipe;
    config cfg;
    colorizer color_map;
    Mat raw, tmp,  out;

    init_rs(&pipe, &cfg);
    auto window = init_gui(&vibrators, &slices, &bins);

    //serial port = serial(PORT, method);

    while (waitKey(1) < 0 && getWindowProperty(window, WND_PROP_AUTOSIZE) >= 0) {

        // recalculate at every frame, cuz it might change on runtime
        h_step = HEIGHT / slices;
        w_step = WIDTH / vibrators;
        d_step = MAX_DISTANCE / bins;

        // poll for frames
        frameset data = pipe.wait_for_frames();
        frame depth = data.get_depth_frame();

        if(display){
            frame colorized = depth.apply_filter(color_map);
            raw = Mat(Size(WIDTH, HEIGHT), CV_8UC3, (void *) colorized.get_data(), Mat::AUTO_STEP);
            tmp = Mat(Size(WIDTH, HEIGHT), CV_8UC3, Scalar(0, 0, 0));
            out = Mat(Size(WIDTH, HEIGHT), CV_8UC3, Scalar(0, 0, 0));

            // draw grid on input image
            for (int i = 0; i < HEIGHT; i += h_step)    line(raw, Point(0, i), Point(WIDTH, i), Scalar(0, 255, 255));
            for (int i = 0; i < WIDTH; i += w_step)     line(raw, Point(i, 0), Point(i, HEIGHT), Scalar(255, 0, 255));
        }

        // send it to the averaging layer
        average_depth(depth, tmp, display);

        if (display) {
            Mat dst;
            hconcat(raw, tmp, dst);
            hconcat(dst, out, dst);
            imshow(window, dst);
        }
    }
    return EXIT_SUCCESS;

} catch (const rs2::error & e){
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e){
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}