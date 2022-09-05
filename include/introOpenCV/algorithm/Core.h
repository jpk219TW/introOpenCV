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
class Core
{
public:
    /**
     * @brief constructor
     */
    Core();
    /**
     * @brief destructor
     */
    ~Core();

private:
    /**
     * @brief Calculate the target heading angle and set the rotor to move to the desired direction
     * @param timerEvent ROS timer event
     */
    void mainCallback(const ros::TimerEvent &timerEvent);
    /**
     * @brief Update current color image
     *
     * @param pColorMsg
     */
    void colorImageCallback(const sensor_msgs::Image::ConstPtr &pColorMsg);
    /**
     * @brief Capture topic data
     */
    void snapData();

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
    ros::Subscriber m_ColorSub;
    /**
     * @brief Timer for running the main callback
     */
    ros::Timer m_CallbackTimer;

    /**
     * @brief Mutex(Critical section: color image)
     */
    std::mutex m_ColorImageMutex;

private:
    /**
     * @brief Color frame
     */
    cv::Mat m_ColorFrame;
    /**
     * @brief Current Color frame
     */
    cv::Mat m_CurrentColorFrame;
    /**
     * @brief Topic name for subscribing color image
     */
    std::string m_ColorName;

    ros::Publisher m_ImagePublisher;

    int m_CallbackIndex;
};