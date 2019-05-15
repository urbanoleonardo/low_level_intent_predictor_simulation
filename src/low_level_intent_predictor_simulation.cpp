#include "LowLevelIntentPredictorSimulator.h"

int main(int argc, char **argv)
{
    LowLevelIntentPredictorSimulator simulator;

    bool ret = simulator.Initialize();
    if (!ret)
    {
        std::cout << "Could not start low level intent predictor simulation." << std::endl;
        return -1;
    }

    simulator.Run();

    simulator.PlotRightHandBeliefs();
    simulator.PlotLeftHandBeliefs();
    simulator.PlotRobotBeliefs();

    return 0;
}