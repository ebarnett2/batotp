////////////////////////////////////////////////////////////////////////////
//! \file    util.h
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief Declares some simple time, file IO, and template functions
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////

#ifndef UTIL_H
#define UTIL_H

#ifdef _MSC_VER
#pragma warning(disable : 4996) // fscanf deprecated
#pragma warning(disable : 4244) // double to float conversion
#if _MSC_PLATFORM_TOOLSET < 140
#include "stdint.h"
#endif
#else
#include <sys/stat.h>
#endif

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include <array>
#include <algorithm> // std::min
#include <functional> // transform,bind1st
#include <cmath>
#include <cstdio>
#include <string>

struct Time
{
   int64_t a;
   int64_t b;
};

// general time functions
Time getTime(void);
double diffTime(const Time &endTime,const Time &startTime);

// general file IO functions
int NextLine(FILE *fid);
int doesFileExist(const char *fname);
int mkDirIfNec(const char *dirname);
std::string readChar(FILE *fid,int &readCountTotal);
int readInt(FILE *fid,int &readCountTotal);
double readDouble(FILE *fid,int &readCountTotal);
std::vector<double> readDoubleVector(FILE *fid,
                                     int &readCountTotal,
                                     const int vectorLen);
bool readBool(FILE *fid,int &readCountTotal);


// general vector functions
int normalizeArcLength(std::vector<double> &s);
int smooth(std::vector<double>&x,int w);
int minsmooth(std::vector<double>&x,int w);
int decimate(std::vector<double>&x,int w);
int solveQuadratic(const double A,
                   const double B,
                   const double C,
                   double &sol1,
                   double &sol2);
double findMedian(std::vector <double> &x);
bool solveLinSys(const std::vector<std::vector<double>> &Av,
             const std::vector<double> &bv,
             std::vector<double> &xv,
             const bool isSVD);
int remClosePts(std::vector<std::vector<double> > &x,
                std::vector<std::vector<double> >& y,
                double xThresh);
std::array<double,4> aa2q(std::array<double,3> aa);
std::array<double,3> q2aa(std::array<double,4> q);


// TEMPLATE FUNCTIONS

// signum or sign function (sgn(0)=0)
template <typename T> int sgn(T val) {
   return (T(0) < val) - (val < T(0));
}

// multiply vector by constant
template <typename T>
std::vector<T> operator*(const T C, std::vector<T>& a)
{
   std::transform(a.begin(), a.end(), a.begin(),
                  std::bind1st(std::multiplies<T>(),C));
   return a;
}

// add constant to vector
template <typename T>
std::vector<T> operator+(const T C, std::vector<T>& a)
{
   std::transform(a.begin(), a.end(), a.begin(),[&C](double x){return x+C;});
   return a;
}

// element-wise add two vectors
template <typename T>
std::vector<T> operator+(std::vector<T>& a,std::vector<T>& b)
{
   int n=(int)a.size();
   std::vector<T> c;
   for(int i=0;i<n;i++) c[i]=a[i]+b[i];
   return c;
}

// element-wise subtract two vectors
template <typename T>
std::vector<T> operator-(const T C, std::vector<T>& a)
{
   std::transform(a.begin(), a.end(), a.begin(),[&C](double x){return x-C;});
   return a;
}

// element-wise add two arrays
template <typename T, size_t N>
std::array<T,N> operator+( std::array<T,N>& a, std::array<T,N>& b)
{
   std::array<T,N> c;
   for(int i=0;i<(int)N;i++) c[i]=a[i]+b[i];
   return c;
}

// 2-norm of an array
template <typename T, size_t N>
double norm( std::array<T,N>& a)
{
   double sumSQ=0.0;
   for(int i=0;i<(int)N;i++) sumSQ+=a[i]*a[i];
   return std::sqrt(sumSQ);
}

// multiply array by constant
template <typename T, size_t N>
std::array<T,N> operator*(const T C,std::array<T,N>& a)
{
   std::transform(a.begin(), a.end(), a.begin(),
                  std::bind1st(std::multiplies<T>(),C));
   return a;
}

#endif // UTIL_H
