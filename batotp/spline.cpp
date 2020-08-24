////////////////////////////////////////////////////////////////////////////
//! \file    spline.cpp
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief Spline class functions
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "spline.h"
#include <cassert>

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief Spline
//!
//! Constructor
//!
//////////////////////////////////////////////////////////////////////////////
Spline::Spline(void)
{
}
//////////////////////////////////////////////////////////////////////////////
//!
//! \brief Spline
//!
//! Destructor
//!
//////////////////////////////////////////////////////////////////////////////
Spline::~Spline(void)
{
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief findInterpSegs
//!
//! find the segments of the old sites aIn that the new sites aOut lie within
//!
//////////////////////////////////////////////////////////////////////////////
int Spline::findInterpSegs(const std::vector<double> &aIn, const std::vector<double> &aOut,\
  splineSegs &mySegs)
{
  int i,seg;
  double aOuti;
  int nPtsIn =(int)aIn.size();
  int nPtsOut=(int)aOut.size();
  std::vector<double> den(nPtsIn);

  mySegs.seg.resize(nPtsOut);
  mySegs.tau.resize(nPtsOut);
  int CurSegIn=0;

  // segments are identified
  for(int i=0;i<nPtsOut;i++)
  {
    aOuti=aOut[i];
    while(1)
    {
      if(aOuti<aIn[CurSegIn+1] || CurSegIn==nPtsIn-2)
        {mySegs.seg[i]=CurSegIn; break;}
      CurSegIn++;
    }
  }

  for(i=0;i<nPtsIn-1;i++)
  {
    den[i]=aIn[i+1]-aIn[i];
    if(den[i]<1e-20)
    {
      printf("Error: division by zero in findInterpSegs().\n");
      return -1;
    }
  }

  // segment fractions (tau) are found for the new sites
  for(int i=0;i<nPtsOut;i++)
  {
    seg=mySegs.seg[i];
    mySegs.tau[i]=(aOut[i]-aIn[seg])/den[seg];
  }

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief interp1linear
//!
//! one-dimensional linear interpolation
//!
//////////////////////////////////////////////////////////////////////////////
int Spline::interp1linear(std::vector<double> &b,const splineSegs &mySegs)
{
  int i,curSeg;
  int nPtsOut=(int)mySegs.seg.size();
  std::vector<double> bOut(nPtsOut);
  for(i=0;i<nPtsOut;i++)
  {
    curSeg=mySegs.seg[i];
    bOut[i]=b[curSeg]+(b[curSeg+1]-b[curSeg])*mySegs.tau[i];
  }
  b=bOut;
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief interp1spline
//!
//! one-dimensional spline interpolation
//!
//////////////////////////////////////////////////////////////////////////////
int Spline::interp1spline(std::vector<double> &b,std::vector<double> &bD,\
  std::vector<double> &bD2,splineCoeffs &bC, const splineSegs &mySegs, const double tfact)
{
  int i,j;
  int nPtsOut=(int)mySegs.seg.size();
  double c0,c1,c2,c3;
  double tau,tau2,tau3;

  b.resize(nPtsOut);
  bD.resize(nPtsOut);
  bD2.resize(nPtsOut);

  double vfact=1.0/tfact;
  double afact=vfact*vfact;
  for(i=0;i<nPtsOut;i++)
  {
    j  =mySegs.seg[i];
    tau=mySegs.tau[i];
    tau2=tau*tau; tau3=tau2*tau;
    c3=bC.c3[j]; c2=bC.c2[j]; c1=bC.c1[j]; c0=bC.c0[j];
    b[i]  =   c3*tau3 +   c2*tau2 + c1*tau + c0;
    bD[i] =(3*c3*tau2 + 2*c2*tau  + c1)*vfact;
    bD2[i]=(6*c3*tau  + 2*c2)*afact;
  }

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief getSplineCoeffs
//!
//! for data values y defined at sites x, find the polynomial coefficients for
//! each segment of a cubic spline interpolant
//!
//! See algorithm details here:
//! http://pages.intnet.mu/cueboy/education/notes/numerical/cubicsplineinterpol.pdf
//!
//////////////////////////////////////////////////////////////////////////////
int Spline::getSplineCoeffs(const std::vector<double>&y,splineCoeffs&yC,const std::string endCond)
{

  unsigned int i;
  size_t nPts = y.size();

  assert(nPts > 0);

  if(nPts != yC.c0.size())
  {
    yC.c0.resize(nPts);
    yC.c1.resize(nPts);
    yC.c2.resize(nPts);
    yC.c3.resize(nPts);
  }

  std::vector<double> sol(nPts);
  for(i=1; i < nPts-1; i++)
  {
     sol[i]=6*(y[i-1]-2*y[i]+y[i+1]);
  }

  if(endCond=="clamped") solveTriDiagClamped(sol);
  else
  {
    if(endCond=="natural")
    {
       solveTriDiagNatural(sol);
    }
    else
    {
      printf("getSplineCoeffs() error: endCond was not \"clamped\" or \"natural\".");
      return -1;
    }
  }
  for(i=0;i<nPts-1;i++)
  {
    yC.c3[i]=(sol[i+1]-sol[i])/6.0;
    yC.c2[i]=sol[i]/2.0;
    yC.c1[i]=y[i+1]-y[i]-(sol[i+1]+2*sol[i])/6.0;
    yC.c0[i]=y[i];
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief solveTriDiagClamped
//!
//! Solve tridiagonal system of equations using the Tridiagonal matrix
//! algorithm (aka the Thomas algorithm) for clamped end conditions
//!
//! See algorithm details here:
//! https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
//! https://en.wikibooks.org/wiki/Algorithm_Implementation/Linear_Algebra/Tridiagonal_matrix_algorithm
//!
//////////////////////////////////////////////////////////////////////////////
int Spline::solveTriDiagClamped(std::vector<double>& d)
{
  int i,n=(int)d.size();
  double a=1.0;
  std::vector<double> c(n,1.0);
  std::vector<double> b(n,4.0); b[0]=2.0; b[n-1]=2.0;
  c[0] /= b[0];
  d[0] /= b[0];

  for (i = 1; i < n; i++)
  {
    c[i] /= b[i] - a*c[i-1];
    d[i] = (d[i] - a*d[i-1]) / (b[i] - a*c[i-1]);
  }

  for (i = n-2; i-- > 0;) d[i] -= c[i]*d[i+1];

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief solveTriDiagNatural
//!
//! Solve tridiagonal system for a spline with natural end conditions
//!
//////////////////////////////////////////////////////////////////////////////
int Spline::solveTriDiagNatural(std::vector<double>& d)
{
  size_t n= d.size()-1;
  assert(n > 2);

  double a=1.0,b=4.0;
  std::vector<double> c(n, 1.0);

  c[1] /= b;
  d[1] /= b;

  for (size_t i = 2; i < n; i++)
  {
    c[i] /= b - a*c[i-1];
    d[i] = (d[i] - a*d[i-1]) / (b - a*c[i-1]);
  }

  d[n] = (d[n] - a*d[n-1]) / (b - a*c[n-1]);

  for (size_t i = n; i > 1; --i)
  {
     d[i-1] -= c[i-1]*d[i];
  }
  return 0;
}
