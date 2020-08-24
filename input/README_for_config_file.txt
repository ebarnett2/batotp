README for creating config.dat -  a configuration file for bisection algorithm (BA) trajectory planning

Input rules for a config.dat file:
- Don't add any new lines
- Make sure all variables to be read are isolated by white space


// VARIABLE DESCRIPTIONS

robotTypeStr: The type of robot, which must be defined in robot.cpp
isParallel  : 1 for parallel robots, 0 for serial robots
_nJoints    : the number of robot joints
nCart       : the number of Cartesian variables
trajFileName: the input trajectory (or path) file name
isBINfile   : 1 for binary input files, 0 for CSV ASCII files (a CSV output file will be generated if the input format is CSV)

pathType 
  CART for trajectories driven by Cartesian data (joint data will be found using inverse kinematics if needed, so an inverse kinematics function for the robot must be provided)
  JOINT for trajectories driven by joint data (Cartesian data will be found using forward kinematics if needed, so a forward kinematics function for the robot must be provided)
  BOTH for trajectories driven by both types of data (Cartesian and joint data will be treated independently, so no inverse or forward kinematics functions are needed. All Cartesian and joint variables must be provided on the input path. This is useful for imposing both Cartesian and joint constraints on a trajector when a kinematics model is not available or is difficult to implement.)


// CONSTRAINTS
For all constraints, use the same units as those of the input path data and/or the units used in kinematics and dynamics functions in robot.cpp

areJntAnglesDegrees : 1 joint positions are in degrees, 0  positions are not in degrees (radians, metres, etc.)
isJntVelConOn : 1 joint velocity constraints are on, 0 joint velocity contstraints are off
JntVelLims    : joint velocity constraints 
isJntAccConOn : 1 joint accel. constraints are on, 0 joint accel. contstraints are off
JntAccLims    : joint acceleration constraints 
isTrqConOn    : 1 joint gen. force constraints are on (a dynamic model is required in robot.cpp), 0 joint gen. force constraints are off
JntTrqMax     : maximum joint torque constraints 
JntTrqMin     : minimum joint torque constraints (NANs are interpreted as -JntTrqMax values)
isCartVelConOn: 1 Cartesian velocity constraints are on, 0 Cartesian velocity contstraints are off
CartVelLims   : Cartesian velocity constraints 
isCartAccConOn: 1 Cartesian accel. constraints are on, 0 Cartesian accel. contstraints are off
CartAccLims   : Cartesian acceleration constraints 


// INTEGRATION PARAMETERS
For all non-time parameters, use the same units as in the input path file

integRes    : The integration time step used for forward and backward integrations
maxIntegTime: The maximum allowed time for integration 

// OTHER CONTROLS
inputDecimFact: decimation factor for input data (data are smoothed using a moving-average window of width inputDecimFact before decimation)
smoothWindow  : number of points to use in smooth and minsmooth operations (applied after decimation)
is_sdotOut    : Controls whether or not the s-sdot curves obtained after each integration are stored and then written to an output file
jntThresh     : threshold in joint units for comparison to zero
cartThresh    : threshold in Cartesian units for comparison to zero
sWeights      : weights assigned for interpolating input path data [input-nodes joint-position-norm Cartesian-position-norm]
scaleType     : scale s according to: 0 input-nodes, 1 joint-position-norm, 2 Cartesian-position-norm
thetaNormRes  : initial joint resolution used for interpolation
thetaNormRes2 : final joint resolution used for interpolation
cartNormRes   : initial Cartesian position resolution used for interpolation
thetaNormRes2 : final Cartesian position resolution used for interpolation
outRes        : time resolution for the output trajectory
outSmoothFact : Smoothing factor used to produce output trajectory data (data are interpolated to outRes/outSmoothFact, then smoothed by outSmoothFact, then interpolated to outRes)
isSVD         : 1 use singular-value decomposition to solve linear systems, 0 use LU decomposition to solve linear systems
isPar2Ser     : 1 eliminate the A matrix in the dynamics equation for a parallel robot to make the equation have the same form as the eqution for a serial robot, 0 don't eliminate the A matrix and use parallel robot computations throughout the algorithm



