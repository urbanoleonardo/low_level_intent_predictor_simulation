#include "LpvDs.h"


LpvDs::LpvDs(const std::string& path_to_file)
:
    path_to_file_(path_to_file)
{
    LpvDs::Initialize();

    gamma_.resize(state_dimension_);
    gamma_.setZero();

    state_.resize(state_dimension_);
    state_.setZero();

    state_attractor_.resize(state_dimension_);
    state_attractor_.setZero();

    A_.resize(state_dimension_, state_dimension_);
    A_.setZero();
}

Eigen::VectorXd LpvDs::GetStateDerivative(const Eigen::VectorXd& state, const Eigen::VectorXd& state_attractor)
{
    bool state_dimension_ok = true;
    bool state_attractor_dimension_ok = true;
    Eigen::VectorXd state_derivative;
    state_derivative.resize(state_dimension_);

    // Check if inputs have correct dimensions
    if (state.size() != state_dimension_)
    {
        state_dimension_ok = false;
    }

    if (state_attractor.size() != state_dimension_)
    {
        state_attractor_dimension_ok = false;
    }

    if (state_dimension_ok && state_attractor_dimension_ok)
    {
        state_ = state;
        state_attractor_ = state_attractor;

        LpvDs::ComputeA();
        state_derivative = A_ * (state_ - state_attractor_);
    }
    else
    {
        state_derivative.setZero();
    }

    return state_derivative;
}

void LpvDs::Initialize()
{
    int i;
    int j;
    int k;
    std::ifstream file;
    std::string line;

    file.open(path_to_file_);
    if(!file)
    {
        std::cout << "Unable to open the model file \n";
        exit(1);
    }
    else
    {     
        std::getline(file, line);
        number_of_components_ = std::stod(line);

        std::getline(file, line);
        state_dimension_ = std::stod(line);

        priors_.resize(number_of_components_);
        mus_.resize(number_of_components_);
        sigmas_.resize(number_of_components_);
        As_.resize(number_of_components_);
        
        std::getline(file, line);
        sscanf(line.c_str(),"%lf %lf %lf", &priors_[0],&priors_[1],&priors_[2]);
        
        for ( i = 0; i < number_of_components_; i++ )
        {
            mus_[i].resize(state_dimension_);

            std::getline(file, line);
            sscanf(line.c_str(),"%lf %lf %lf", &mus_[i][0],&mus_[i][1],&mus_[i][2]);
        }

        for ( i = 0; i < number_of_components_; i++ )
        {
            sigmas_[i].resize(state_dimension_, state_dimension_);
            for ( j = 0; j < state_dimension_; j++ )
            {
                std::getline(file, line);
                sscanf(line.c_str(),"%lf %lf %lf", &sigmas_[i](j, 0),&sigmas_[i](j, 1),&sigmas_[i](j, 2));
            }
        }

        for ( i = 0; i < number_of_components_; i++ )
        {
            As_[i].resize(state_dimension_, state_dimension_);
            for ( j = 0; j < state_dimension_; j++ )
            {
                std::getline(file, line);
                sscanf(line.c_str(),"%lf %lf %lf", &As_[i](j, 0),&As_[i](j, 1),&As_[i](j, 2));
            }
        }

        file.close();
    }
}

void LpvDs::ComputeGamma()
{
    double sum;
    gamma_.setZero();

	for (int i = 0; i < number_of_components_; i++)
    {
        gamma_[i] = priors_[i] * LpvDs::ComputeGaussianPdf(mus_[i], sigmas_[i]);
    }

    sum = gamma_.sum();

    // If sum is close to zero, gamma the weights are all equal and depend on number_of_components_
	if (sum < 1e-100)
    {
		for (int i = 0; i < number_of_components_; i++)
        {
            gamma_[i] = 1.0 / number_of_components_;
        }
	}
	else
    {
        gamma_ = gamma_ / sum;
    }
}

void LpvDs::ComputeA()
{
    A_.setZero();

	if (number_of_components_ > 1)
    {
        LpvDs::ComputeGamma();
    }
	else
    {
        gamma_[number_of_components_-1] = 1;
    }

    // Compute A_ as linear combination of As_ with weights equal to their respective gamma_
	for (int i = 0; i < number_of_components_; i++)
    {
        A_ = A_ + As_[i]*gamma_[i];
    }
}

double LpvDs::ComputeGaussianPdf(const Eigen::VectorXd& mu, const Eigen::MatrixXd& sigma)
{
    // TODO: Implement Cholezky Decomposition and avoid inverse()
    double a = pow(sigma.determinant(), -0.5) / pow(2*M_PI, 1.5);
    double b = -0.5 * ((state_-mu).transpose()) * sigma.inverse() * (state_ - mu);
    
    return a * exp(b);
}

