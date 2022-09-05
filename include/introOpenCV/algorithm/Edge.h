#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sys/stat.h>

#include <algorithm>
#include <boost/format.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>
class Edge
{
public:
    /**
     * @brief constructor
     */
    Edge();
    /**
     * @brief destructor
     */
    ~Edge();

private:
    /**
     * @brief Calculate the target heading angle and set the rotor to move to the desired direction
     * @param timerEvent ROS timer event
     */
    void mainCallback(const ros::TimerEvent &timerEvent);
    /**
     * @brief Update current color image
     *
     * @param pGrayMsg
     */
    void ImageCallback(const sensor_msgs::Image::ConstPtr &pGrayMsg);
    /**
     * @brief Capture topic data
     */
    void snapData();

    void CannyThreshold(void);

private:
    /**
     * @brief Node handler
     */
    ros::NodeHandle m_NodeHandler;
    /**
     * @brief callback queue
     */
    ros::CallbackQueue m_CustumQueue;
    /**
     * @brief spinner
     */
    ros::AsyncSpinner m_AsyncSpinner;
    /**
     * @brief Subscriber for receiving RGB image
     */
    ros::Subscriber m_GraySub;
    /**
     * @brief Timer for running the main callback
     */
    ros::Timer m_CallbackTimer;

    /**
     * @brief Mutex(Critical section: color image)
     */
    std::mutex m_GrayImageMutex;

private:
    /**
     * @brief Color frame
     */
    cv::Mat m_GrayFrame;
    /**
     * @brief Current Color frame
     */
    cv::Mat m_CurrentGrayFrame;
    /**
     * @brief Topic name for subscribing color image
     */
    std::string m_GrayName;

    ros::Publisher m_ImagePublisher;

    int m_CallbackIndex;

private:
    cv::Mat detected_edges;
    int lowThreshold = 0;
    const int max_lowThreshold = 100;
    const int ratio = 3;
    const int kernel_size = 3;
    const char *window_name = "Edge Map";
};