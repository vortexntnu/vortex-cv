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

void mcLabFilter(const cv::Mat &original, cv::Mat &filtered)
{
	// Erode image to make blacks more black
	cv::Mat eroded;
	erodingFilter(original, eroded, 2);

	// Make an unsharp mask from original image
	size_t blurSize = 30;
	cv::Mat blurred;
	GaussianBlur(original, blurred, cv::Size(2 * blurSize + 1, 2 * blurSize + 1), 0);

	// Compute the unsharp mask
	cv::Mat mask = original - blurred;
	cv::Mat unsharp;

	// Add mask to the eroded image instead of the original
	// Higher weight of mask will create a sharper but more noisy image
	addWeighted(eroded, 1, mask, 5, 0, filtered);
}

void whiteBalanceFilter(const cv::Mat &original, cv::Mat &filtered, double contrastPercentage)
{
	cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
	balance->setP(contrastPercentage);
	balance->balanceWhite(original, filtered);
}