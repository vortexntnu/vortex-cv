#include <image_processing.hpp>

void sharpeningFilter(const cv::Mat &original, cv::Mat &filtered)
{
	// Sharpen image
	cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::filter2D(original, filtered, -1, kernel);
}

void unsharpeningFilter(const cv::Mat &original, cv::Mat &filtered, size_t blurSize, double sharpness)
{
	// Create a blurred version of the image
	cv::Mat blurred;
	GaussianBlur(original, blurred, cv::Size(2 * blurSize + 1, 2 * blurSize + 1), 0);

	// Compute the unsharp mask
	cv::Mat mask = original - blurred;
	cv::Mat unsharp;

	addWeighted(original, 1, mask, 3, 0, filtered);
}

void erodingFilter(const cv::Mat &original, cv::Mat &filtered, size_t erosionSize)
{
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize, erosionSize));

	// Apply erosion to the image
	cv::erode(original, filtered, element);
}

void dilatingFilter(const cv::Mat &original, cv::Mat &filtered, size_t dilationSize)
{
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1), cv::Point(dilationSize, dilationSize));

	// Apply dilation to the image
	cv::dilate(original, filtered, element);
}

void whiteBalanceFilter(const cv::Mat &original, cv::Mat &filtered, double contrastPercentage)
{
	cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
	balance->setP(contrastPercentage);
	balance->balanceWhite(original, filtered);
}




void ebusFilter(const cv::Mat &original, cv::Mat &filtered, size_t erosionSize, size_t blurSize, size_t maskWeight)
{
	// Erode image to make blacks more black
	cv::Mat eroded;
	erodingFilter(original, eroded, erosionSize);

	// Make an unsharp mask from original image
	cv::Mat blurred;
	GaussianBlur(original, blurred, cv::Size(2 * blurSize + 1, 2 * blurSize + 1), 0);

	// Compute the unsharp mask
	cv::Mat mask = original - blurred;
	cv::Mat unsharp;

	// Add mask to the eroded image instead of the original
	// Higher weight of mask will create a sharper but more noisy image
	addWeighted(eroded, 1, mask, maskWeight, 0, filtered);
}


// Must correspond with image_filter_parameters.cfg enum
enum class Filter {
	NoFilter,
    Sharpening,
    Unsharpening,
    Eroding,
    Dilating,
    WhiteBalance,
	Ebus,
};

void filter_from_rqt(const cv::Mat &original, cv::Mat &filtered, image_filters::imgFilterConfig &config)
{
	switch ((Filter)config.filter_type)
	{
		case Filter::NoFilter:
			original.copyTo(filtered);
			break;
		case Filter::Sharpening:
			sharpeningFilter(original, filtered);
			break;
		case Filter::Unsharpening:
			unsharpeningFilter(original, filtered, config.unsharp_blur_size);
			break;
		case Filter::Eroding:
			erodingFilter(original, filtered, config.erosion_size);
			break;
		case Filter::Dilating:
			dilatingFilter(original, filtered, config.dilation_size);
			break;
		case Filter::WhiteBalance:
			whiteBalanceFilter(original, filtered, config.contrast_percentage);
			break;
		case Filter::Ebus:
			ebusFilter(original, filtered, config.ebus_erosion_size, config.ebus_blur_size, config.ebus_unsharp_weight);
			break;
	}
	ROS_WARN_STREAM_COND(filtered.empty(), "FILTER_FROM_RQT: Filtered image empty");
}