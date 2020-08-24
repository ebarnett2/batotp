////////////////////////////////////////////////////////////////////////////
//! \file    ba.h
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief   Declares the ba object
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////

#ifndef BA_H
#define BA_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#if defined(WIN32) || defined(_WIN32) || defined(WIN64) || defined(_WIN64)
#define isWin
#else
#include <cmath>
#endif

#ifdef _MSC_VER
#pragma warning(disable : 4996) // fscanf deprecated
#pragma warning(disable : 4244) // double to float conversion
#endif

#include "config.h"
#include "spline.h"
#include "robot.h"
#include <vector>
#include <array>
#include <algorithm>

namespace BATOTP{

// variables used in the interpSpecial function
struct InterpVars
{
   double tTeachFact;
   double thetaNormFact;
   double cartPosNormFact;
   double sLast;
   double sResNew;
   double sResi;
};

struct Traj
{
   double tresInput; // [s] resolution for the recorded traj.
   double sres;      // resolution of sMVC
   unsigned int nPts;  // Number of points on the traj.
   double tTotalTraj;
   std::string trajFileName;
   std::vector<std::string> trajFileHeader; // header of UR CSV file

   std::vector<double> timestamp;   //nPts input timestamp array

   // ptsOrig is array that indicates the number of points on the original
   // taught trajectory before each point on the current trajectory
   // ptsOrigC holds the spline coefficients for this array
   std::vector<double>  ptsOrig;   //nPts
   Spline::splineCoeffs ptsOrigC;  //nPtsC

   // joint positions and derivatives vs. sC
   std::vector<std::vector<double>> theta;   //nPts
   std::vector<std::vector<double>> thetaD;  //nPts
   std::vector<std::vector<double>> thetaD2; //nPts

   // Cartesian positions and derivatives vs. sC
   std::vector<std::vector<double>> cart;   //nPts
   std::vector<std::vector<double>> cartD;  //nPts
   std::vector<std::vector<double>> cartD2; //nPts

   // dynamic model variables
   std::vector<std::vector<double>> trq; //nPts
   std::vector<std::vector<double>> a1; //nPts
   std::vector<std::vector<double>> a2; //nPts
   std::vector<std::vector<double>> a3; //nPts
   std::vector<std::vector<double>> a4; //nPts

   // the maximum-velocity curve (MVC)
   int curSegMVC; // current segment of the MVC
   double tauMVC; // position on the mvc segment (normalized 0 to 1)
   double sCur;   // current coordinates (sCur,sdotCur) of the MVC
   int i;         // current point on the MVC;

   double sdotCur;
   bool sdotLimTypeT; // indicates if sdot was limited by the MVC
   double sddotH;     // high value of sddot range ouput by bisection method
   double sddotL;     // low value  of sddot range ouput by bisection method

   // length of these three variables is initially nPts, then it changes
   // after integration
   std::vector<double> sMVC;
   std::vector<double> tMVC;
   std::vector<double> sdot;
   std::vector<double> sddot;
   Spline::splineCoeffs sdotC;

   // MVC history, used for outputting to file and then plotting in MATLAB
   struct MVChist
   {
      std::vector<std::vector<double>> s;
      std::vector<std::vector<double>> sdot;
   }myMVChist;

   std::vector<double> thetapt;
   std::vector<double> thetaDpt;
   std::vector<double> thetaD2pt;

   std::vector<double> a1pt;
   std::vector<double> a2pt;
   std::vector<double> a3pt;
   std::vector<double> a4pt;

   // cartpt arrays need to have dim 6 for a general manipulator
   std::vector<double> cartpt;
   std::vector<double> cartDpt;
   std::vector<double> cartD2pt;
   std::array<double,3> CartAccCoeffs;

   std::vector<std::vector<double>> Apt;

   double sLastSec;
   bool isOn_sdot=false;

   // spline interpolants of the joint and Cartesian positions
   int    nPtsC;   // Number of points on each of the splines
   double sresC;   // resolution of sC
   double vFact;   // velocity factor to apply when calculating first-order derivs wrt s
   double aFact;   // accel. factor to apply when calculating  second-order derivs wrt s
   int    curSegC; // current segment of the splines
   double tauC;    // position on the spline segment (normalized 0 to 1)
   std::vector<double> sC; // nPtsC position on the spline
   std::vector<Spline::splineCoeffs> thetaC; //nPtsC
   std::vector<Spline::splineCoeffs> a1C;
   std::vector<Spline::splineCoeffs> a2C;
   std::vector<Spline::splineCoeffs> a3C;
   std::vector<Spline::splineCoeffs> a4C;
   std::vector<Spline::splineCoeffs> cartC;  //nPtsC
};

// static const arrays and vectors added here for MSVC 2013 compatibility
// (non-static member array initialization not allowed)
static const std::vector<double> jntVelLims_init(6, 190);
static const std::vector<double> jntAccLims_init(6, 500);
static const std::vector<double> zeros6(6, 0);
static const std::array<double, 3> sWeights_initArr = { 0, 0.1, 1 };
static const std::vector<double> sWeights_init(sWeights_initArr.begin(), sWeights_initArr.end());

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief The BA class
//!
//////////////////////////////////////////////////////////////////////////////
class BA
{
public:
   // Functions
    BA(void);
   ~BA(void);

    struct Config;
    enum ErrorOptimization{NO_ERROR, MAX_INTEGRATION_TIME};

   // do this in BA with a top-level function?
   int readConfigData(const char* filename);
   int loadConfigData(const Config &conf);
   int loadTrajectoryData(Traj &traj);
   int interpInputData(Traj &myTraj);
   int sweep(Traj &myTraj);
   int interpOutputData(Traj &myTraj);
   int writeOutputData(Traj &myTraj);
   int optimize(Traj &myTraj);

   //setters
   inline void setIsLastSweep(bool isLastSweep){_isLastSweep = isLastSweep;}
   inline void setIntegDir(int integDir){_integDir = integDir;}
   inline void setIsInterpOnly(bool isInterpOnly){_isInterpOnly = isInterpOnly;}
   inline void setCartesianMaximalVelocity(const double &velocity){_CartVelMax = velocity;}
   inline void setCartesianMaximalAcceleration(const double &acceleration){_CartAccMax = acceleration;}
   inline void setJointMaximalVelocity(const std::vector<double> &velocity){_JntVelMax = velocity;}
   inline void setJointMaximalAcceleration(const std::vector<double> &acceleration){_JntAccMax = acceleration;}
   inline void setIsAutoIntegRes(const bool isAutoIntegRes){_isAutoIntegRes = isAutoIntegRes;}
   inline void setHomeFolder(const std::string &HomeFolder) {_HomeFolder = HomeFolder;}
   inline void setInputFolder(const std::string &InputFolder) {_InputFolder = InputFolder;}
   inline void setOutputFolder(const std::string &OutputFolder) {_OutputFolder = OutputFolder;}

   //getters
   inline double getCartesianMaximalVelocity() const {return _CartVelMax;}
   inline double getCartesianMaximalAcceleration() const {return _CartAccMax;}
   inline std::vector<double> getJointMaximalVelocity() const {return _JntVelMax;}
   inline std::vector<double> getJointMaximalAcceleration() const {return _JntAccMax;}
   inline ErrorOptimization getErrorOptimization() const {return _errorOptimization;}
   inline double getOutTimeRes() const {return _outRes;}
   inline std::string getHomeFolder() const {return _HomeFolder;}
   inline std::string getInputFolder() const {return _InputFolder;}
   inline std::string getOutputFolder() const {return _OutputFolder;}

   // Config struct used for loading config. data using loadConfigData() function
   struct Config{
      std::string robotTypeStr = "UR";
      bool isParallelMech = false;
      int nJoints = 6;
      int nCart =6 ;
      std::string trajFileName = "urtraj.csv";
      bool isBinFile = false;
      std::string pathType = "BOTH";

      //Constraints
      bool isJntVelConon = true;
      std::vector<double> jntVelLims = jntVelLims_init;
      bool isJntAccConOn = true;
      std::vector<double> jntAccLims = jntAccLims_init;
      bool isTrqConOn = false;
      std::vector<double> jntTrqMax = zeros6;
      std::vector<double> jntTrqMin = zeros6;
      bool isCartVelConOn = true;
      double cartVelMax = 0.4;
      bool isCarAccConOn = true;
      double cartAccMax = 5.0;

      // INTEGRATION PARAMETERS
      double integRes = 0.016;
      double maxIntegTime = 60000;

      // OTHER CONTROLS
      int inputDecimFact = 1;
      int smoothWindow = 1;
      bool is_sdotOut = false;
      double jntThresh = 1e-6;
      double cartThresh = 1e-6;
      std::vector<double> sWeights = sWeights_init;
      int scaleType = 2;
      double thetaNormRes = 0.01;
      double thetaNormRes2 = 0.01;
      double cartNormRes = 0.002;
      double cartNormRes2 = 0.002;
      double outRes = 0.008;
      int outSmoothFact = 1;
      bool isSVD = false;
      bool isPar2Ser = false;
   };

private:
   // See input/config.dat and input/README_for_config_file.txt for a detailed
   // description of the variables

   std::string _robotTypeStr;  // robot type
   bool _isParallelMech;
   unsigned int _nJoints;
   unsigned int _nCart;
   std::string  _trajFileName;
   bool _isBINfile;
   int _pathType;

   // CONSTRAINTS
   bool _areJointAnglesDegrees=false;
   bool _isJntVelConOn;
   std::vector<double> _JntVelMax; // length nJoints
   bool _isJntAccConOn;
   std::vector<double> _JntAccMax; // length nJoints
   bool _isTrqConOn;
   std::vector<double> _JntTrqMax; // length nJoints
   std::vector<double> _JntTrqMin; // length nJoints
   bool _isCartVelConOn;
   double _CartVelMax; // max. Cartesian speed
   bool _isCartAccConOn;
   double _CartAccMax; // max. Cartesian accel.

   // INTEGRATION PARAMETERS
   double _integRes;     // [s] integration resolution
   double _maxIntegTime; // [s] maximum integration time

   // OTHER CONTROLS
   int    _inputDecimFact; // [points] input data decimation factor
   int    _smoothWindow;   // [points] number of points to use in smooth and minsmooth operations
   bool is_sdotOut;
   double _jntThresh;
   double _cartThresh;
   std::vector<double> _sWeights;
   int _scaleType;
   double _thetaNormRes;
   double _thetaNormRes2;
   double _cartNormRes;
   double _cartNormRes2;
   double _outRes;       // [s] resolution for the output traj.
   double _outSmoothFact;
   bool _isSVD;
   bool _isPar2Ser;

   //----------------------------------------------------------
   // Other class variables (not read from the config.dat file)
   bool _isInterpOnly=false; // interpolate traj. data with no optimization
   bool _isParallelMechOrig; // bool to indicate if parallel mech. is
   bool _isGenericRobot; // bool to indicate if there is no kinematic model and no cart. data in the input traj. file
   bool _isAutoIntegRes=true; // determine _integRes automatically
   bool _isInterpolated=false; // bool to indicate if input data has been interpolated
   bool _isLastSweep; // bool to indicate if sweep() function is being called for the last (second) time

   int    _robotType; //1=KUKA, 2=UR, 3=RR, 4=CSPR3DOF, 5=GENJNT
   int _integDir; // 1=forward, -1=reverse
   int _dynDim; // number of eqs. in the dynamics model system of eqs.

   double _CartMomMax; // max. Cartesian momentum (not currently used)
   double _quadraticRadThresh; // _cartThresh^2
   double _sdotMin = std::numeric_limits<double>::max(); // minimum change in sdot during integration (this is overridden in sweep())

   // Butcher tableau for Runge-Kutta numerical integration
   std::array<std::array<double,6>,6> _B;
   std::array<double,6> _A;
   std::array<double,6> _dA;
   std::array<double,7> _E;

   std::string _HomeFolder;
   std::string _InputFolder;
   std::string _OutputFolder;

   ErrorOptimization _errorOptimization;

   // Functions
   inline void setErrorOptimization(const ErrorOptimization &errorOptimization)
     {_errorOptimization = errorOptimization;}

   int evalSplineFullTraj(Traj &myTraj, const double oldRes, double newRes);
   int adjust_s(Traj &traj,std::string interpType);
   int interpSpecial(Traj &myTraj,const InterpVars &myInterpVars);
   int findDynModel(Traj &myTraj);
   int dynCoeffs2Ser(std::vector<std::vector<double>> &A,
                     std::vector<std::vector<double>> &b, bool _isSVD, int trajPt);
   int aa2qVect(std::vector<std::vector<double>> &pose);
   int q2aaVect(std::vector<std::vector<double>> &pose);

   // sweep()
   bool verifySecondOrderConstraints(Traj &myTraj,
                                     const double sdotCur,
                                     const double sddotMax);
   int applyAccelConstraintsBisectionPt(Traj &myTraj,double &sddot,int &nIter);
   int sdotLim(Traj &myTraj, double &sdot, const std::string &type);
   int updateCurSeg(const std::vector<double> &s, const double sCur,
                    int &curSeg, double &tau);
   int evalSplinePartials(Traj &myTraj);
   int evalCartQuadCoeffs(Traj &myTraj);
   double evalsdot(Traj &myTraj, const std::string &type);

   int trajReadBIN(Traj &myTraj, const char* filename);
   int trajReadCSV(Traj &myTraj, const char* filename);

   int printInputData(const Traj &myTraj);
   int trajWriteBIN(Traj &myTraj, const char *fname);
   int trajWriteCSV(Traj &myTraj, const char *fname);
   int sdotWrite(Traj &myTraj, const char *fname);
   int interpTrajLinear(Traj &traj,const int nPtsNew);

   // Classes
   Spline mySpline;
   Robot  myRobot;
};

}//namespace BATOTP

#endif //BA_H
