#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

class ImagePreprocessing {
private:
	double clahe_clip_limit{2};
	double clahe_tile_grid_size{8};

	cv::Mat lab_image, dst; // TODO: Consider moving this into useClahe function
	cv::Ptr<cv::CLAHE> clahe_;
	cv::Mat image_clahe; // TODO: Consider moving this into useClahe function

public:
	/**
	 * @brief Construct a new Image Preprocessing object
	 *
	 * @param init_clahe_cliplimit Number of pixels used to clip the CDF for
	 * histogram equalization
	 * @param init_clahe_tilegridsize clahe_grid_size_ x clahe_grid_size_ pixel
	 * neighborhood used
	 */
	ImagePreprocessing(double init_clahe_cliplimit, double init_clahe_tilegridsize);

	/**
	 * @brief
	 *
	 * @param cv_image_
	 * @param clahe_plane Image channel to use CLAHE algorithm on
	 * @return cv::Mat
	 */
	cv::Mat doClahe(cv::Mat cv_image_, int clahe_plane);
};