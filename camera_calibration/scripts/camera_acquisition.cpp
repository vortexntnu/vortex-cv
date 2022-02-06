 // Standard includes
#include <string.h>

// OpenCV include (for display)
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {   
    cv::VideoCapture zed;
    if (argc == 2)
        zed.open(atoi(argv[1]));
    else
        zed.open(0);

    //define the camera resolution
    cv::Size size_sbs(1920 * 2, 1080);

    // change camera param to fit requested resolution (ZED camera gives side by side images)
    zed.set(cv::CAP_PROP_FRAME_WIDTH, size_sbs.width);
    zed.set(cv::CAP_PROP_FRAME_HEIGHT, size_sbs.height);

    // create a file to save images names
    std::vector<std::string> v_names;

    // alloc a mat to store acquired images
    cv::Mat imag_sbs(size_sbs, CV_8UC3);
    int w_ = size_sbs.width * .5;

    // define Left and Right reference Mat
    cv::Mat imL = imag_sbs(cv::Rect(0, 0, w_, size_sbs.height));
    cv::Mat imR = imag_sbs(cv::Rect(w_, 0, w_, size_sbs.height));

    int nb_save = 0;
    const int NB_REQUIRED = 20;
    while (nb_save < NB_REQUIRED) {

        // grab and retrieve the current image
        zed >> imag_sbs;

        // Left and Right mat are directly updated because they are ref.

        cv::imshow("Left", imL); // display left image
        auto k = cv::waitKey(30);

        // if Space-bar is pressed, save the image
        if (k == 32) {
            std::string im_name("zed_image_L" + std::to_string(nb_save) + ".png");
            cv::imwrite(im_name, imL);
            v_names.push_back(im_name);

            im_name = "zed_image_R" + std::to_string(nb_save) + ".png";
            cv::imwrite(im_name, imR);
            v_names.push_back(im_name);

            nb_save++;
            std::cout << "Save im " << nb_save << "/" << NB_REQUIRED << std::endl;
        }
    }
    // close file and camera
    cv::FileStorage fs("zed_image_list.xml", cv::FileStorage::WRITE);
    fs.write("images", v_names);
    fs.release();
    zed.release();
    return EXIT_SUCCESS;
}
