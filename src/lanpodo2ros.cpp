#include "lanpodo2ros.h"

LANPODO2ROS::LANPODO2ROS()
{
    size = sizeof(podo2ros);
    buffer = new char[size];
    sock = 0;
}

LANPODO2ROS::~LANPODO2ROS()
{
    delete [] buffer;
    buffer = nullptr;
}
