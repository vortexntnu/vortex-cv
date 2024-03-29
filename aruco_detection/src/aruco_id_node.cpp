#include "aruco_id_node.hpp"

ArucoIdNode::ArucoIdNode() : loop_rate{10}
{
	dictionary = new cv::aruco::Dictionary;
	// dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100); //homemade docking plate
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL); // TAC

	opImageSub = node.subscribe("filtered_image", 10, &ArucoIdNode::callback, this);
	opImagePub = node.advertise<sensor_msgs::Image>("aruco_image", 100);
	opIdPub    = node.advertise<std_msgs::Int64>("pipeline_id", 100);
}

void ArucoIdNode::callback(const sensor_msgs::ImageConstPtr &img_source)
{
	const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);

	cv::Mat img = cvImage->image;

	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;

	// Detect markers
	cv::aruco::detectMarkers(img, dictionary, corners, ids);
	// Draw markers on image and publish image
	cv::aruco::drawDetectedMarkers(img, corners, ids);
	publishCVImg(img, cvImage->header.stamp);

	if (ids.size() == 0)
		return;

	// Save found markers
	if (ids.size() > 1)
	ROS_WARN("More than one id detected!");
	int foundId = ids.at(0);

	// Ignore duplicate ids
	if (std::find(storedIds.begin(), storedIds.end(), foundId) != storedIds.end())
		return; // No new id detected
	
	publishId(foundId);
	ROS_INFO_STREAM("Found id: " << foundId);
	storedIds.push_back(foundId);

	static std::string time = stampToString(ros::Time::now());
	writeIntsToFile("/home/eirik/pipelineIds/foundIds_" + time + ".csv", storedIds);
}

void ArucoIdNode::publishCVImg(const cv::Mat &img, ros::Time timestamp)
{
	static size_t counter{0};
	cv_bridge::CvImage imgBridge;
	sensor_msgs::Image imgMsg;

	std_msgs::Header header;
	header.seq   = counter++;
	header.stamp = timestamp;

	imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
	imgBridge.toImageMsg(imgMsg);
	opImagePub.publish(imgMsg);
}

void ArucoIdNode::publishId(int id)
{
	std_msgs::Int64 msg{};
	msg.data = id;
	opIdPub.publish(msg);
}

void ArucoIdNode::execute()
{
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ArucoIdNode");

	ArucoIdNode arucoNode;
	arucoNode.execute();
}