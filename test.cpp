#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 7);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd z = Eigen::VectorXd::Ones(4);
    
    Eigen::VectorXd t = z - H * x;

    std::cout << t << std::endl;

    return 0;
}