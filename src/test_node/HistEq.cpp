#include "introOpenCV/algorithm/Core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "histEQ");
    Core m_Core;
    ros::spin();
    return 0;
}