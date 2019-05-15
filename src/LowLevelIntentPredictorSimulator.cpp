#include "LowLevelIntentPredictorSimulator.h"

LowLevelIntentPredictorSimulator::LowLevelIntentPredictorSimulator()
{

}

bool LowLevelIntentPredictorSimulator::Initialize()
{
    LowLevelIntentPredictorSimulator::InitializeTestSettings();
    LowLevelIntentPredictorSimulator::InitializeTest();
    
    LowLevelIntentPredictorSimulator::InitializeHands();
    LowLevelIntentPredictorSimulator::InitializeRobotBeliefs();

    LowLevelIntentPredictorSimulator::InitializePlots();

    return true;
}

void LowLevelIntentPredictorSimulator::Run()
{
    
    for (int i = 0; i < num_steps_; i++)
    {
        // Compute right hand and left hand beliefs
        bh_r_ = right_hand_->ComputeBeliefs(x_r_test_.col(i), dx_r_test_.col(i), xo_r_test_.col(i), xo_l_test_.col(i), x_0_test_.col(i));
        bh_l_ = left_hand_->ComputeBeliefs(x_l_test_.col(i), dx_l_test_.col(i), xo_r_test_.col(i), xo_l_test_.col(i), x_0_test_.col(i));

        // Compute robot beliefs
        br_[0] = bh_r_[1] * bh_l_[1] + bh_r_[2] * bh_l_[1] + bh_r_[1] * bh_l_[2];
        br_[1] = bh_r_[0] * bh_l_[0] + bh_r_[2] * bh_l_[0] + bh_r_[0] * bh_l_[2];
        br_[2] = bh_r_[1] * bh_l_[0] + bh_r_[0] * bh_l_[1] + bh_r_[2] * bh_l_[2];

        // Create variables for plots
        plot_t_.push_back(i * dt_);
        plot_bh_rr_.push_back(bh_r_[0]);
        plot_bh_rl_.push_back(bh_r_[1]);
        plot_bh_r0_.push_back(bh_r_[2]);
        plot_bh_lr_.push_back(bh_l_[0]);
        plot_bh_ll_.push_back(bh_l_[1]);
        plot_bh_l0_.push_back(bh_l_[2]);
        plot_br_r_.push_back(br_[0]);
        plot_br_l_.push_back(br_[1]);
        plot_br_0_.push_back(br_[2]);
    }
}

void LowLevelIntentPredictorSimulator::PlotRightHandBeliefs()
{
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::named_plot("$b^H_{rr}$", plot_t_, plot_bh_rr_);
    matplotlibcpp::named_plot("$b^H_{rl}$", plot_t_, plot_bh_rl_);
    matplotlibcpp::named_plot("$b^H_{r0}$", plot_t_, plot_bh_r0_);
    matplotlibcpp::xticks(plot_x_ticks_);
    matplotlibcpp::yticks(plot_y_ticks_);
    matplotlibcpp::xlabel("t");
    matplotlibcpp::title("Right Hand Beliefs");
    matplotlibcpp::legend();
    matplotlibcpp::save("./output/bh_r.png");
}

void LowLevelIntentPredictorSimulator::PlotLeftHandBeliefs()
{
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::named_plot("$b^H_{lr}$", plot_t_, plot_bh_lr_);
    matplotlibcpp::named_plot("$b^H_{ll}$", plot_t_, plot_bh_ll_);
    matplotlibcpp::named_plot("$b^H_{l0}$", plot_t_, plot_bh_l0_);
    matplotlibcpp::xticks(plot_x_ticks_);
    matplotlibcpp::yticks(plot_y_ticks_);
    matplotlibcpp::xlabel("t");
    matplotlibcpp::title("Left Hand Beliefs");
    matplotlibcpp::legend();
    matplotlibcpp::save("./output/bh_l.png");
}

void LowLevelIntentPredictorSimulator::PlotRobotBeliefs()
{
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::named_plot("$b^R_{r}$", plot_t_, plot_br_r_);
    matplotlibcpp::named_plot("$b^R_{l}$", plot_t_, plot_br_l_);
    matplotlibcpp::named_plot("$b^R_{0}$", plot_t_, plot_br_0_);
    matplotlibcpp::xticks(plot_x_ticks_);
    matplotlibcpp::yticks(plot_y_ticks_);
    matplotlibcpp::xlabel("t");
    matplotlibcpp::title("Robot Beliefs");
    matplotlibcpp::legend();
    matplotlibcpp::save("./output/br.png");
}

void LowLevelIntentPredictorSimulator::InitializeTestSettings()
{
    dt_ = 0.005;
    n_w_ = 20;
    epsilon_ = 10000;
    x_dim_ = 3;
    num_steps_ = 415;
    num_att_ = 3;

    test_set_path_ = "../test/test.txt";
    right_hand_model_path_ = "../human_task_models/hand_right_model.txt";
    left_hand_model_path_ = "../human_task_models/hand_left_model.txt";

}

void LowLevelIntentPredictorSimulator::InitializeTest()
{
    int i;
    int j;
    double temp_value;
    std::ifstream file;
    std::vector<double> temp_vector;

    file.open(test_set_path_);
    if(!file)
    {
        std::cout << "Unable to open the file \n";
        exit(1);
    }
    else
    {
        while(!file.eof())
        {
            file >> temp_value;
            temp_vector.push_back(temp_value);
        }
        xo_r_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[0], x_dim_, num_steps_);
        xo_l_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[x_dim_*num_steps_], x_dim_, num_steps_);
        x_0_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[2*x_dim_*num_steps_], x_dim_, num_steps_);
        x_r_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[3*x_dim_*num_steps_], x_dim_, num_steps_);
        dx_r_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[4*x_dim_*num_steps_], x_dim_, num_steps_);
        x_l_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[5*x_dim_*num_steps_], x_dim_, num_steps_);
        dx_l_test_ = Eigen::Map<Eigen::MatrixXd>(&temp_vector[6*x_dim_*num_steps_], x_dim_, num_steps_);

        file.close();
    }
}

void LowLevelIntentPredictorSimulator::InitializeRobotBeliefs()
{
    br_.resize(num_att_);
}

void LowLevelIntentPredictorSimulator::InitializeHands()
{
    right_hand_ = new Hand(right_hand_model_path_, dt_, x_dim_, n_w_, num_att_);
    left_hand_ = new Hand(left_hand_model_path_, dt_, x_dim_, n_w_, num_att_);
}

void LowLevelIntentPredictorSimulator::InitializeHumanTaskModels()
{
    right_hand_model_ = new LpvDs(right_hand_model_path_);
    left_hand_model_ = new LpvDs(left_hand_model_path_);
}

void LowLevelIntentPredictorSimulator::InitializePlots()
{
    int i;

    plot_dx_ = 0.2;
    plot_dy_ = 0.2;
    plot_x_max_ = 2.2;
    plot_y_max_ = 1.4;

    // Fill x_ticks
    for (i = 0; i < (int) std::ceil(plot_x_max_ / plot_dx_); i++)
    {
        plot_x_ticks_.push_back((double) i * plot_dx_);
    }

    // Fill y_ticks
    for (i = 0; i < (int) std::ceil(plot_y_max_ / plot_dy_); i++)
    {
        plot_y_ticks_.push_back((double) i * plot_dy_);
    }
}