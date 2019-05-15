#include "Hand.h"

Hand::Hand(const std::string& task_model_path, const double& dt, const int& x_dim, const int& win_len, const int& num_att)
:
    task_model_path_(task_model_path),
    dt_(dt),
    x_dim_(x_dim),
    win_len_(win_len),
    num_att_(num_att),
    lfd_model_(false),
    epsilon_(10000)
{
    Hand::InitializeTaskModel();
    Hand::InitializeWindows();
    Hand::InitializeTaskSimulations();

    Hand::InitializeBeliefUpdates();
    Hand::InitializeBeliefs();
}

std::vector<double> Hand::ComputeBeliefs(const Eigen::MatrixXd& x_new, const Eigen::MatrixXd& dx_new, const Eigen::MatrixXd& xo_r_new, const Eigen::MatrixXd& xo_l_new, const Eigen::MatrixXd& x_0_new)
{
    std::vector<double> bh(num_att_);
    std::vector<double> bh_updates(num_att_);

    Hand::UpdateWindows(x_new, dx_new);
    Hand::UpdateTaskAttractors(xo_r_new, xo_l_new, x_0_new);

    Hand::ComputeTaskSimulations();
    Hand::ComputeBeliefUpdates();

    // Makes sure beliefs are all between zero and one
    bh[0] = b_r_;
    bh[1] = b_l_;
    bh[2] = b_0_;
    bh_updates[0] = b_r_update_;
    bh_updates[1] = b_l_update_;
    bh_updates[2] = b_0_update_;
    
    for (int i = 0; i < num_att_; i++)
    {           
        bh[i] += bh_updates[i] * dt_;

        if (bh[i] > 1)
        {
            bh[i] = 1;
        }

        if (bh[i] < 0)
        {
            bh[i] = 0;
        }
    }
    b_r_ = bh[0];
    b_l_ = bh[1];
    b_0_ = bh[2];

    return bh;  
}

void Hand::InitializeTaskModel()
{
    task_model_ = new LpvDs(task_model_path_);
}

void Hand::InitializeBeliefUpdates()
{
    b_r_update_ = 0;
    b_l_update_ = 0;
    b_0_update_ = 0;
}
void Hand::InitializeBeliefs()
{
    b_r_ = 0;
    b_l_ = 0;
    b_0_ = 1;
}

void Hand::InitializeWindows()
{   
    Eigen::VectorXd zero(x_dim_);
    zero.setZero();
    
    for (int i = 0; i < win_len_; i++)
    {
        // hand position window
        x_win_.push_front(zero);

        // hand velocity window
        dx_win_.push_front(zero);
    }
}

void Hand::InitializeTaskSimulations()
{
    // State derivatives average values
    dx_win_avg_.resize(x_dim_);
    dx_win_avg_.setZero();

    // Simulation towards right handle
    x_r_sim_0_.resize(x_dim_);
    x_r_sim_0_.setZero();
    x_r_sim_.resize(x_dim_);
    x_r_sim_.setZero();

    // Simulation towards left handle
    x_l_sim_0_.resize(x_dim_);
    x_l_sim_0_.setZero();
    x_l_sim_.resize(x_dim_);
    x_l_sim_.setZero();

    // Simulation towards idle state (hip)
    x_0_sim_0_.resize(x_dim_);
    x_0_sim_0_.setZero();
    x_0_sim_.resize(x_dim_);
    x_0_sim_.setZero();

    // Prediction
    x_d_.resize(x_dim_);
    x_d_.setZero();
}

void Hand::UpdateWindows(const Eigen::MatrixXd& x_new, const Eigen::MatrixXd& dx_new)
{
    Eigen::VectorXd zero(x_dim_);
    zero.setZero();

    x_win_.pop_back();
    dx_win_.pop_back();

    x_win_.push_front(x_new);
    dx_win_.push_front(dx_new);

    dx_win_avg_ = std::accumulate(dx_win_.begin(), dx_win_.end(), zero) / win_len_;

}

void Hand::UpdateTaskAttractors(const Eigen::MatrixXd& xo_r_new, const Eigen::MatrixXd& xo_l_new, const Eigen::MatrixXd& x_0_new)
{
    xo_r_ = xo_r_new;
    xo_l_ = xo_l_new;
    x_0_ = x_0_new;
}

void Hand::ComputeTaskSimulations()
{
    Eigen::VectorXd dx_r_curr(x_dim_);
    Eigen::VectorXd dx_l_curr(x_dim_);
    Eigen::VectorXd dx_0_curr(x_dim_);
    Eigen::VectorXd x_r_att(x_dim_);
    Eigen::VectorXd x_l_att(x_dim_);
    Eigen::VectorXd x_0_att(x_dim_);
    x_r_att = xo_r_-x_win_.back();
    x_l_att = xo_l_-x_win_.back();
    x_0_att = x_0_-x_win_.back();

    x_r_sim_0_ = x_win_.back();
    x_l_sim_0_ = x_win_.back();
    x_0_sim_0_ = x_win_.back();
    x_r_sim_ = x_r_sim_0_;
    x_l_sim_ = x_l_sim_0_;
    x_0_sim_ = x_0_sim_0_;
    for (int i = 0; i < win_len_; i++)
    {
        // True if I want to simulate beliefs using approaching motion models learned from demonstration
        // False if I want to simulate beliefs using simple linear model
        if (lfd_model_)
        {
            dx_r_curr = task_model_->GetStateDerivative(x_r_sim_, x_r_att);
            dx_l_curr = task_model_->GetStateDerivative(x_l_sim_, x_l_att);
        }
        else
        {
            dx_r_curr = x_r_att - x_r_sim_0_;
            dx_l_curr = x_l_att - x_l_sim_0_;
        }
        dx_0_curr = x_0_att - x_0_sim_0_;

        x_r_sim_ += dt_ * (dx_r_curr/dx_r_curr.norm()) * dx_win_avg_.norm();
        x_l_sim_ += dt_ * (dx_l_curr/dx_l_curr.norm()) * dx_win_avg_.norm();
        x_0_sim_ += dt_ * (dx_0_curr/dx_0_curr.norm()) * dx_win_avg_.norm();

        // Avoids that simulations go further than the attractor
        if ((x_r_att-x_r_sim_).norm() < 1e-3)
        {
            x_r_sim_ = x_r_att;
        }
        if ((x_l_att-x_l_sim_).norm() < 1e-3)
        {
            x_l_sim_ = x_l_att;
        }
        if ((x_0_att-x_0_sim_).norm() < 1e-3)
        {
            x_0_sim_ = x_0_att;
        }

    }

}

void Hand::ComputeBeliefUpdates()
{
    int max_update_index;
    int max_first_index;
    double max_update;
    double max_first;
    double max_second;
    double max_mean;
    double sum = 0;
    std::vector<double> bh(num_att_);
    std::vector<double> bh_updates(num_att_);
    std::vector<double> buffer(num_att_);

    // Compute prediction
    x_d_ = b_r_ * x_r_sim_ + b_l_ * x_l_sim_ + b_0_ * x_0_sim_;

    // Compute belief updates
    b_r_update_ = epsilon_ * (x_win_.front() - x_win_.back() - x_d_).transpose() * x_r_sim_;
    b_l_update_ = epsilon_ * (x_win_.front() - x_win_.back() - x_d_).transpose() * x_l_sim_;
    b_0_update_ = epsilon_ * (x_win_.front() - x_win_.back() - x_d_).transpose() * x_0_sim_;

    bh[0] = b_r_;
    bh[1] = b_l_;
    bh[2] = b_0_;
    bh_updates[0] = b_r_update_;
    bh_updates[1] = b_l_update_;
    bh_updates[2] = b_0_update_;

    /* Winner take all process - BEGIN */
    // Find index of max belief update
    max_update = *std::max_element(bh_updates.begin(), bh_updates.end());
    max_update_index = std::distance(bh_updates.begin(), std::max_element(bh_updates.begin(), bh_updates.end()));

    // If its corresponding belief is already 1, set belief updates to zero
    if (std::abs(bh[max_update_index] - 1.) < 1e-6)
    {
        std::fill(bh_updates.begin(), bh_updates.end(), 0.);
    }
    else
    {
        // Compute mean value of two max updates
        buffer = bh_updates;
        max_first = *std::max_element(buffer.begin(), buffer.end());
        max_first_index = std::distance(buffer.begin(), std::max_element(buffer.begin(), buffer.end()));
        buffer.erase(buffer.begin()+max_first_index);
        max_second = *std::max_element(buffer.begin(), buffer.end());
        max_mean = (max_first + max_second) / 2.;

        for (int i = 0; i < bh_updates.size(); i++)
        {
            bh_updates[i] -= max_mean;
        }

        // Computing sum of positive updates
        sum = 0.;
        for (int i = 0; i < bh_updates.size(); i++)
        {
            if (std::abs(bh[i]) > 1e-6 || bh_updates[i] > 0)
            {
                sum += bh_updates[i];
            }
        }
        bh_updates[max_update_index] -= sum;
    }
    /* Winner take all process - END */  

    b_r_update_ = bh_updates[0];
    b_l_update_ = bh_updates[1];
    b_0_update_ = bh_updates[2];

}