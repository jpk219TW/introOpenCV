#include "introOpenCV/algorithm/Core.h"

Core::Core()
    : m_NodeHandler(), m_CustumQueue(), m_AsyncSpinner(0, &m_CustumQueue), m_CallbackIndex(0)
{
    m_NodeHandler.setCallbackQueue(&m_CustumQueue);
    m_AsyncSpinner.start();
    m_ColorSub = m_NodeHandler.subscribe<sensor_msgs::Image>("/twinny_cam/color/image_raw", 10,
                                                             &Core::colorImageCallback, this);
    // Turn on the main callback timer
    m_CallbackTimer = m_NodeHandler.createTimer(ros::Duration(0.033), &Core::mainCallback, this);

    m_ImagePublisher = m_NodeHandler.advertise<sensor_msgs::Image>("/hist_equalized_image", 10);
}
Core::~Core()
{
}

void Core::colorImageCallback(const sensor_msgs::Image::ConstPtr &pColorMsg)
{
    std::unique_lock<std::mutex> lock(m_ColorImageMutex);
    cv_bridge::CvImagePtr colorPtr;
    colorPtr = cv_bridge::toCvCopy(pColorMsg, sensor_msgs::image_encodings::BGR8);
    m_ColorFrame = colorPtr->image;
}

void Core::mainCallback(const ros::TimerEvent &timerEvent)
{
    // Freeze data
    snapData();
    m_CallbackIndex++;

    if (m_CurrentColorFrame.empty() == true) {
        std::cout << "Image is EMPTY. RETURN. Idx: " << m_CallbackIndex << "\n";
        return;
    }

    // Now the image is gray
    cv::cvtColor(m_CurrentColorFrame, m_CurrentColorFrame, cv::COLOR_BGR2GRAY);

    // Hist eq.
    cv::Mat dst;
    cv::equalizeHist(m_CurrentColorFrame, dst);

    // std::string scnResultPath = "/home/twinny/resultSave/";
    // mkdir(scnResultPath.c_str(), 0776);

    // auto imageNumString = std::string(8 - std::to_string(m_CallbackIndex).length(), '0') +
    //                       std::to_string(m_CallbackIndex);

    // auto imageName = scnResultPath + imageNumString + ".jpg";

    // cv::imwrite(imageName, dst);

    ////////////////////////////////////////////////////////////////////////
    // Publish rviz msg
    cv_bridge::CvImage imageMsg;
    // bbMsg.header.frame_id = m_RobotFrameID;
    imageMsg.header.frame_id = "map";
    imageMsg.header.stamp = ros::Time::now();
    imageMsg.header.seq = m_CallbackIndex;
    imageMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    imageMsg.image = dst.clone();

    m_ImagePublisher.publish(imageMsg.toImageMsg());
}

void Core::snapData()
{
    std::unique_lock<std::mutex> lockColor(m_ColorImageMutex);
    // Color
    m_CurrentColorFrame = m_ColorFrame.clone();
}
