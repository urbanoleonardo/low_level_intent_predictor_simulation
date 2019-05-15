#ifndef LPVDS_H
#define LPVDS_H

#include <iostream>
#include <fstream>

#include <Eigen/Eigen>

class LpvDs
{

public:

    LpvDs(const std::string& path_to_file);

    Eigen::VectorXd GetStateDerivative(const Eigen::VectorXd& state, const Eigen::VectorXd& state_attractor);

protected:

    int number_of_components_;
    int state_dimension_;

    std::string path_to_file_;

    Eigen::VectorXd priors_;
    Eigen::VectorXd gamma_;
    Eigen::VectorXd state_;
    Eigen::VectorXd state_attractor_;

    Eigen::MatrixXd A_;
    // Lpv parameters
    std::vector<Eigen::VectorXd> mus_;
    std::vector<Eigen::MatrixXd> sigmas_;
    std::vector<Eigen::MatrixXd> As_;

    void Initialize();
    void ComputeGamma();
    void ComputeA();

    double ComputeGaussianPdf(const Eigen::VectorXd& mu, const Eigen::MatrixXd& sigma);

};

#endif // LPVDS_H