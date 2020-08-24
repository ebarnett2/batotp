////////////////////////////////////////////////////////////////////////////
//! \file    robot.h
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief   Robot-specific functions
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOT_H
#define ROBOT_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <vector>

namespace BATOTP{

// robot type
static const int KUKA  =1;
static const int UR    =2;
static const int RR    =3;
static const int CSPR3DOF  =4;
static const int GENJNT=5;

// path type
static const int JOINT=1;
static const int CART =2;
static const int BOTH =3;

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief The Robot class
//!
//////////////////////////////////////////////////////////////////////////////
class Robot
{
public:
    int call_set_robotType(const std::string &robotTypeStr);
    int call_fwdKin(const std::vector<std::vector<double>> &theta,
                    std::vector<std::vector<double>> &cart);
    int call_invKin(std::vector<std::vector<double>> &theta,
                    const std::vector<std::vector<double>> &cart);

    int call_dynSerial(std::vector<std::vector<double>> &a1,
                       std::vector<std::vector<double>> &a2,
                       std::vector<std::vector<double>> &a3,
                       std::vector<std::vector<double>> &a4,
                       const std::vector<std::vector<double>> &theta,
                       const std::vector<std::vector<double>> &thetaD,
                       const std::vector<std::vector<double>> &thetaD2);
    int call_dynParallel(std::vector<std::vector<double>> &a1,
                         std::vector<std::vector<double>> &a2,
                         std::vector<std::vector<double>> &a3,
                         std::vector<std::vector<double>> &a4,
                         const std::vector<std::vector<double>> &cart,
                         const std::vector<std::vector<double>> &cartD,
                         const std::vector<std::vector<double>> &cartD2);
    int call_setA(const std::vector<double> &theta,
                  const std::vector<double> &cart,
                  std::vector<std::vector<double>> &A);
private:
    int _robotType;
    std::string _robotTypeStr;
    std::vector<std::vector<double>> _pmat;

    int set_robotType(const std::string &robotTypeStr);

    // Forward Kinematics
    int fwdKin(const std::vector<std::vector<double> >& theta,
               std::vector<std::vector<double>> &cart);
    int fwdKinKuka(const std::vector<std::vector<double>> &theta,
                   std::vector<std::vector<double>> &cart);
    int fwdKinRR(const std::vector<std::vector<double>> &theta,
                 std::vector<std::vector<double>> &cart);

    //Inverse Kinematics
    int invKin(std::vector<std::vector<double>> &theta,
               const std::vector<std::vector<double> >& cart);
    int invKinCSPR3DOF(std::vector<std::vector<double>> &theta,
                       const std::vector<std::vector<double>> &cart);
    int findCSPR3DOFpmat(std::vector<std::vector<double>> &pmat);

    // Serial robot dynamics
    int dynSerial(std::vector<std::vector<double>> &a1,
                  std::vector<std::vector<double>> &a2,
                  std::vector<std::vector<double>> &a3,
                  std::vector<std::vector<double>> &a4,
                  const std::vector<std::vector<double>> &theta,
                  const std::vector<std::vector<double>> &thetaD,
                  const std::vector<std::vector<double>> &thetaD2);
    int dynRR(std::vector<std::vector<double>> &a1,
              std::vector<std::vector<double>> &a2,
              std::vector<std::vector<double>> &a3,
              std::vector<std::vector<double>> &a4,
              const std::vector<std::vector<double>> &theta,
              const std::vector<std::vector<double>> &thetaD,
              const std::vector<std::vector<double>> &thetaD2);

    //Parallel robot dynamics
    int dynCSPR3DOF(std::vector<std::vector<double>> &a1,
                    std::vector<std::vector<double>> &a2,
                    std::vector<std::vector<double>> &a3,
                    std::vector<std::vector<double>> &a4,
                    const std::vector<std::vector<double>> &cartD,
                    const std::vector<std::vector<double>> &cartD2);
    int dynParallel(std::vector<std::vector<double>> &a1,
                    std::vector<std::vector<double>> &a2,
                    std::vector<std::vector<double>> &a3,
                    std::vector<std::vector<double>> &a4,
                    const std::vector<std::vector<double>> &cart,
                    const std::vector<std::vector<double>> &cartD,
                    const std::vector<std::vector<double>> &cartD2);

    int setA(const std::vector<double> &theta,
             const std::vector<double> &cart,
             std::vector<std::vector<double>> &A);

};
}//namespace BATOTP

#endif // ROBOT_H
