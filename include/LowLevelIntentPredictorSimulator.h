#ifndef LOWLEVELINTENTPREDICTORSIMULATOR_H
#define LOWLEVELINTENTPREDICTORSIMULATOR_H

#include "Hand.h"
#include "LpvDs.h"

#include "matplotlibcpp.h"

class LowLevelIntentPredictorSimulator
{

public:

    LowLevelIntentPredictorSimulator();
    
    bool Initialize();

    void Run();
    void PlotRightHandBeliefs();
    void PlotLeftHandBeliefs();
    void PlotRobotBeliefs();

protected:
    
    int n_w_;
    int num_steps_;
    int x_dim_;
    int num_att_;

    double dt_;
    double epsilon_;
    double plot_dx_;
    double plot_dy_;
    double plot_x_max_;
    double plot_y_max_;

    std::string test_set_path_;
    std::string right_hand_model_path_;
    std::string left_hand_model_path_;

    Eigen::MatrixXd xo_r_test_;
    Eigen::MatrixXd xo_l_test_;
    Eigen::MatrixXd x_0_test_;
    Eigen::MatrixXd x_r_test_;
    Eigen::MatrixXd dx_r_test_;
    Eigen::MatrixXd x_l_test_;
    Eigen::MatrixXd dx_l_test_;

    std::vector<double> bh_r_;
    std::vector<double> bh_l_;
    std::vector<double> bh_0_;
    std::vector<double> br_;
    std::vector<double> plot_bh_rr_;
    std::vector<double> plot_bh_rl_;
    std::vector<double> plot_bh_r0_;
    std::vector<double> plot_bh_lr_;
    std::vector<double> plot_bh_ll_;
    std::vector<double> plot_bh_l0_;
    std::vector<double> plot_br_r_;
    std::vector<double> plot_br_l_;
    std::vector<double> plot_br_0_;
    std::vector<double> plot_t_;
    std::vector<double> plot_x_ticks_;
    std::vector<double> plot_y_ticks_;

    Hand *right_hand_;
    Hand *left_hand_;

    LpvDs *right_hand_model_;
    LpvDs *left_hand_model_;
    
    void InitializeTestSettings();
    void InitializeTest();
    void InitializeRobotBeliefs();
    void InitializeHands();
    void InitializeHumanTaskModels();
    void InitializePlots();

};

#endif