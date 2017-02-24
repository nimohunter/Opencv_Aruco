#ifndef TRANSDATA_H
#define TRANSDATA_H


class TransData
{
public:
    TransData(double vx, double vy, double vz, double rx, double ry, double rz) {
        setTransData(vx, vy, vz, rx, ry, rz);
    }

    char* getTransData() ;
    void setTransData(double vx, double vy, double vz, double rx, double ry, double rz);


private:
    double v_x,v_y,v_z,r_x,r_y,r_z;
    char buffer[256];

};

#endif // TRANSDATA_H
