#include "lanros2podo.h"

LANROS2PODO::LANROS2PODO()
{
    size = sizeof(command);
    buffer = new char[size];
    sock = 0;
}

LANROS2PODO::~LANROS2PODO()
{
    delete [] buffer;
    buffer = nullptr;
}
