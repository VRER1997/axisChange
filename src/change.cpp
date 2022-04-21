#include <Eigen/Dense>
#include <Eigen/Core>

#include <Eigen/src/Core/Matrix.h>
#include <fstream>
#include <ostream>
#include <sstream>
#include <vector>
#include <iostream>

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x; 
    R_x <<
            1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;
    R_y <<
            cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;
    R_z <<
            cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

bool isRotationMatirx(Eigen::Matrix3d R)
{
    double err=1e-3;
    Eigen::Matrix3d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
    return (shouldIdenity - I).norm() < err;
}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}

int main (int argc, char **argv) {

    // Read in the file
    std::ifstream file("../data/05.txt");
    std::ofstream outfile("/home/lab/source/KITTI_odometry_evaluation_tool/data/05_pred.txt");
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream lineStream(line);

        std::vector<double> data;
        while (lineStream.good()) {
            std::string substr;
            std::getline(lineStream, substr, ' ');
            data.push_back(std::stod(substr));
        }

        std::cout << data[3] << " " << data[7] << " " << data[11] << std::endl;

        // chage trans_x, trans_y, trans_z
        double trans_x = -data[7];
        double trans_y = -data[11];
        double trans_z = data[3];

        Eigen::Matrix3d rot;
        rot << data[0], data[1], data[2],
               data[4], data[5], data[6],
               data[8], data[9], data[10];

        // rotation matrix to ouler angle
        // Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);
        Eigen::Vector3d euler = rotationMatrixToEulerAngles(rot);

        // chage ouler angle order
        Eigen::Vector3d euler_new;
        euler_new << euler[0], -euler[2], euler[1]; // yaw pitch pitch

        // oular angle to rotation matrix
        Eigen::Matrix3d rot2 = eulerAnglesToRotationMatrix(euler_new);
        // rot2 = Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitZ()) // yaw
        //         * Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitY()) // roll
        //         * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()); // pitch

        
        // std::cout << rot << std::endl;
        outfile << rot2(0, 0) << " " << rot2(0, 1) << " " << rot2(0, 2) << " " << trans_x << " "
                << rot2(1, 0) << " " << rot2(1, 1) << " " << rot2(1, 2) << " " << trans_y << " "
                << rot2(2, 0) << " " << rot2(2, 1) << " " << rot2(2, 2) << " " << trans_z << '\n';  
        // break;
    }

    return 0;
}