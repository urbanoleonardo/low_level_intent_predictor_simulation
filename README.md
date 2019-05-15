

# Low-Level Intent Predictor Simulation

A dynamical systems approach to human intent recognition based on the hands motion. 

## Content
```
low_level_intent_predictor_simulator
│    README.md
│    CMakeLists.txt
│
└─── doc
│	problem_formulation.pdf
│
└─── human_task_models
│       hand_left_model.txt
│       hand_right_model.txt
│       model_template.txt
│    
└─── test
│       test.txt
│       test_template.txt
│    
└─── build
│	│    
│	└─── output
│    
└─── src
│       low_level_intent_predictor_simulation.cpp
│       LowLevelIntentPredictorSimulator.cpp


# Low-Level Intent Predictor Simulation

A dynamical systems approach to human intent recognition based on the hands motion. 

## Content
```
low_level_intent_predictor_simulator
│    README.md
│    CMakeLists.txt
│
└─── doc
│	problem_formulation.pdf
│
└─── human_task_models
│       hand_left_model.txt
│       hand_right_model.txt
│       model_template.txt
│    
└─── test
│       test.txt
│       test_template.txt
│    
└─── build
│	│    
│	└─── output
│    
└─── src
│       low_level_intent_predictor_simulation.cpp
│       LowLevelIntentPredictorSimulator.cpp
│	Hand.cpp
│       LpvDs.cpp
│    
└─── include
	LowLevelIntentPredictorSimulator.h
	Hand.h
	LpvDs.h
	matplotlibcpp.h
```

## Dependencies (Linux)
This repository depends on [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page). You can install it by typing the following command in your terminal:
```
sudo apt-get install libeigen3-dev
```
Finally, please check the [matplotlib-cpp](https://github.com/lava/matplotlib-cpp) page to make sure all its dependencies are satisfied. 
## Running the code

Run the following terminal commands from the pakage repository.
```
cd build
cmake ..
make
./low_level_intent_predictor_simulator
```
## Input
1. **test/test.txt**
	This file contains data describing the motion of the two hands of the human going toward both of the handles. 
	The structure of this file is described in test_template.txt .
2. **human_task_models/hand_right_model.txt**
	This file contains the data describing the model learned from demonstrations of the approaching motion of the right hand toward one of the two handles. The model is represented via a Linear Parameter Varying System (LPV) as in [1].
	The structure of this file is described in model_template.txt .
3. **human_task_models/hand_left_model.txt**
This file contains the data describing the model learned from demonstrations of the approaching motion of the left hand toward one of the two handles. The model is represented via a LPV System as in [1].
The structure of this file is described in model_template.txt .
## Output
1. **build/output/bh_r.png**
	This image represents the evolution of the beliefs of the right hand of the human over the data described in test.txt.
	
2. **build/output/bh_l.png**
	This image represents the evolution of the beliefs of the left hand of the human over the data described in test.txt.
3. **build/output/br.png**
This image represents the evolution of the beliefs of the robot over the data described in test.txt.


### References     
> [1] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL).

## Authors

 **Leonardo Urbano** - Doctoral Assistant in Robotics, Control and Intelligent Systems at [LASA](http://lasa.epfl.ch/), EPFL.
│	Hand.cpp
│       LpvDs.cpp
│    
└─── include
	LowLevelIntentPredictorSimulator.h
	Hand.h
	LpvDs.h
	matplotlibcpp.h
```

## Running the code

Run the following terminal commands from the pakage repository.
```
cd build
cmake ..
make
./low_level_intent_predictor_simulator
```
## Input
1. **test/test.txt**
	This file contains data describing the motion of the two hands of the human going toward both of the handles. 
	The structure of this file is described in test_template.txt .
2. **human_task_models/hand_right_model.txt**
	This file contains the data describing the model learned from demonstrations of the approaching motion of the right hand toward one of the two handles. The model is represented via a Linear Parameter Varying System (LPV) as in [1].
	The structure of this file is described in model_template.txt .
3. **human_task_models/hand_left_model.txt**
This file contains the data describing the model learned from demonstrations of the approaching motion of the left hand toward one of the two handles. The model is represented via a LPV System as in [1].
The structure of this file is described in model_template.txt .
## Output
1. **build/output/bh_r.png**
	This image represents the evolution of the beliefs of the right hand of the human over the data described in test.txt.
	
2. **build/output/bh_l.png**
	This image represents the evolution of the beliefs of the left hand of the human over the data described in test.txt.
3. **build/output/br.png**
This image represents the evolution of the beliefs of the robot over the data described in test.txt.


### References     
> [1] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL).

## Authors

 **Leonardo Urbano** - Doctoral Assistant in Robotics, Control and Intelligent Systems at [LASA](http://lasa.epfl.ch/), EPFL.
