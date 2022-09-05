#include "introOpenCV/algorithm/Edge.h"

Edge::Edge()
    : m_NodeHandler(), m_CustumQueue(), m_AsyncSpinner(0, &m_CustumQueue), m_CallbackIndex(0)
{
    m_NodeHandler.setCallbackQueue(&m_CustumQueue);
    m_AsyncSpinner.start();
    m_GraySub = m_NodeHandler.subscribe<sensor_msgs::Image>("/hist_equalized_image", 10,
                                                            &Edge::ImageCallback, this);
    // Turn on the main callback timer
    m_CallbackTimer = m_NodeHandler.createTimer(ros::Duration(0.033), &Edge::mainCallback, this);

    // m_ImagePublisher = m_NodeHandler.advertise<sensor_msgs::Image>("/hist_equalized_image", 10);
}
Edge::~Edge()
{
}

void Edge::ImageCallback(const sensor_msgs::Image::ConstPtr &pGrayMsg)
{
    std::unique_lock<std::mutex> lock(m_GrayImageMutex);
    cv_bridge::CvImagePtr grayPtr;
    grayPtr = cv_bridge::toCvCopy(pGrayMsg, sensor_msgs::image_encodings::TYPE_8UC1);
    m_GrayFrame = grayPtr->image;

    // std::string scnResultPath = "/home/twinny/histimage/";
    // mkdir(scnResultPath.c_str(), 0776);
    // //
    // auto imageNumString = std::string(8 - std::to_string(m_CallbackIndex).length(), '0') +
    //                       std::to_string(m_CallbackIndex);
    // //
    // auto imageName = scnResultPath + imageNumString + ".jpg";
    // //
    // cv::imwrite(imageName, m_GrayFrame);
}

void Edge::mainCallback(const ros::TimerEvent &timerEvent)
{
    // Freeze data
    snapData();
    m_CallbackIndex++;

    if (m_CurrentGrayFrame.empty() == true) {
        std::cout << "Image is EMPTY. RETURN. Idx: " << m_CallbackIndex << "\n";
        return;
    }

    // Now the image is gray -- not needed for edge detection
    // cv::cvtColor(m_CurrentColorFrame, m_CurrentColorFrame, cv::COLOR_BGR2GRAY);

    // Hist eq.
    // cv::Mat dst;
    // cv::equalizeHist(m_CurrentColorFrame, dst);
    cannyThreshold();

    ////////////////////////////////////////////////////////////////////////

    std::string scnResultPath = "/home/twinny/finalresult/";
    mkdir(scnResultPath.c_str(), 0776);

    auto imageNumString = std::string(8 - std::to_string(m_CallbackIndex).length(), '0') +
                          std::to_string(m_CallbackIndex);

    auto imageName = scnResultPath + imageNumString + ".jpg";

    cv::imwrite(imageName, m_Detected_edges);
}

void Edge::snapData()
{
    std::unique_lock<std::mutex> lockGray(m_GrayImageMutex);
    // Color
    m_CurrentGrayFrame = m_GrayFrame.clone();
}

void Edge::cannyThreshold(void)
{
    cv::blur(m_CurrentGrayFrame, m_Detected_edges, cv::Size(3, 3));
    cv::Canny(m_Detected_edges, m_Detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
    // dst = Scalar::all(0);
    // src.copyTo(dst, m_Detected_edges);
    // imshow(window_name, dst);
}
