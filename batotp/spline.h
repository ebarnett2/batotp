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
//! \brief Declares the spline object
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////

#ifndef SPLINE_H
#define SPLINE_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <vector> // needed for STL vectors
#if !defined(_MSC_VER)
   #include "stdio.h"
   #include <string>
#endif

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief The Spline class
//!
//////////////////////////////////////////////////////////////////////////////
class Spline
{
public:
   // Variables
   struct splineCoeffs
   {
      std::vector<double> c0;
      std::vector<double> c1;
      std::vector<double> c2;
      std::vector<double> c3;
   };
   struct splineSegs
   {
      std::vector<int> seg;
      std::vector<double> tau;
   };

   // Functions
   Spline(void);
   ~Spline(void);
   int getSplineCoeffs(const std::vector<double>&y, splineCoeffs &yC,const std::string endCond);
   int findInterpSegs(const std::vector<double> &aIn, const std::vector<double> &aOut,\
                      splineSegs &mySegs);
   int interp1linear(std::vector<double> &b,const splineSegs &mySegs);
   int interp1spline(std::vector<double> &b,std::vector<double> &bD,\
                     std::vector<double> &bD2,splineCoeffs &bC, const splineSegs &mySegs, const double tfact);
private:
   //Functions
   int solveTriDiagNatural(std::vector<double>& d);
   int solveTriDiagClamped(std::vector<double>& d);
   int resizeSplineCoeffs(splineCoeffs &x, const int nPts);
};

#endif //SPLINE_H
