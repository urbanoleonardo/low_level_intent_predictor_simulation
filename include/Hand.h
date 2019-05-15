#ifndef HAND_H
#define HAND_H

#include <deque>
#include <numeric>

#include "LpvDs.h"

class Hand
{

public:

    Hand(const std::string& task_model_path, const double& dt, const int& x_dim, const int& win_len, const int& num_att);

    std::vector<double> ComputeBeliefs(const Eigen::MatrixXd& x_new, const Eigen::MatrixXd& dx_new, const Eigen::MatrixXd& xo_r_new, const Eigen::MatrixXd& xo_l_new, const Eigen::MatrixXd& x_0_new);

protected:

    // True to compute beliefs using model learned from demonstrations
    // Currently, it works better without (False)
    bool lfd_model_; 

    int win_len_;
    int num_att_;
    int x_dim_;

    double dt_;
    double epsilon_;
    double b_r_update_;
    double b_l_update_;
    double b_0_update_;
    double b_r_;
    double b_l_;
    double b_0_;

    std::string task_model_path_;

    Eigen::VectorXd x_;
    Eigen::VectorXd dx_;
    Eigen::VectorXd x_d_;
    Eigen::VectorXd dx_win_avg_;
    Eigen::VectorXd x_r_sim_0_;
    Eigen::VectorXd x_l_sim_0_;
    Eigen::VectorXd x_0_sim_0_;
    Eigen::VectorXd x_r_sim_;
    Eigen::VectorXd x_l_sim_;
    Eigen::VectorXd x_0_sim_;
    Eigen::VectorXd xo_r_;
    Eigen::VectorXd xo_l_;
    Eigen::VectorXd x_0_;

    std::deque<Eigen::VectorXd> x_win_;
    std::deque<Eigen::VectorXd> dx_win_;

    std::vector<std::vector<double>> beliefs_for_plot_;

    LpvDs *task_model_;

    void InitializeTaskModel();
    void InitializeWindows();
    void InitializeTaskSimulations();
    void InitializeBeliefUpdates();
    void InitializeBeliefs();
    void UpdateWindows(const Eigen::MatrixXd& x_new, const Eigen::MatrixXd& dx_new);
    void UpdateTaskAttractors(const Eigen::MatrixXd& xo_r_new, const Eigen::MatrixXd& xo_l_new, const Eigen::MatrixXd& x_0_new);
    void ComputeTaskSimulations();
    void ComputeBeliefUpdates();

};

#endif
