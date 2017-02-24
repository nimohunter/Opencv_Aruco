#include "transdata.h"
#include <stdio.h>
#include <stdlib.h>



char *TransData::getTransData()
{
    sprintf(buffer, "%lf_%lf_%lf_%lf_%lf_%lf", v_x, v_y, v_z, r_x, r_y, r_z);
    //sprintf(buffer, "%d", 123);
    return buffer;
}

void TransData::setTransData(double vx, double vy, double vz, double rx, double ry, double rz)
{
    v_x = vx;
    v_y = vy;
    v_z = vz;
    r_x = rx;
    r_y = ry;
    r_z = rz;
}
