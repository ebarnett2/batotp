////////////////////////////////////////////////////////////////////////////
//! \file    util.cpp
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief Time, file IO, and general std::vector functions
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////


#if defined(WIN32) || defined(_WIN32) || defined(WIN64) || defined(_WIN64)
  #define isWin
#endif

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "util.h"
#ifdef _MSC_VER
#define NOMINMAX // for std::min
#endif
#ifdef isWin
  #include <windows.h> // for cpu time tracking
  #include <io.h> // for function doesFileExist()
#else //Linux
  #include "stdio.h" //for file IO
  #include <unistd.h> // for function doesFileExist(): for testing for file existence
#endif

#include <Eigen/Dense>
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief getTime
//!
//! get the current time (for computation time tracking)
//!
//////////////////////////////////////////////////////////////////////////////
Time getTime(void)
{
  Time time;
  #ifdef isWin
     LARGE_INTEGER count,freq;
     QueryPerformanceCounter(&count);
     QueryPerformanceFrequency(&freq);
     time.a=(int64_t)count.QuadPart;
     time.b=(int64_t)freq.QuadPart;
  #else
    struct timespec CurrentTime;
    clock_gettime(CLOCK_REALTIME, &CurrentTime);
    time.a=(int64_t)CurrentTime.tv_sec;
    time.b=(int64_t)CurrentTime.tv_nsec;
  #endif
  return time;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief diffTime
//!
//! Compute the difference between two time structs (for computation time tracking)
//!
//////////////////////////////////////////////////////////////////////////////
double diffTime(const Time &endTime,const Time &startTime)
{
  double dTime;
  #ifdef isWin
    dTime=(double)(endTime.a-startTime.a)/(double)startTime.b;
  #else
    if(endTime.b < startTime.b)
         dTime=(double)(endTime.a-startTime.a)-1+(double)(endTime.b-startTime.b+1.0e9)/1.0e9;
    else dTime=(double)(endTime.a-startTime.a)  +(double)(endTime.b-startTime.b)/1.0e9;
  #endif
  return dTime;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief NextLine
//!
//!Rread in characters one at a time from a file until end-of-line or
//! end-of-file is reached
//!
//////////////////////////////////////////////////////////////////////////////
int NextLine(FILE *fid)
{
  int c=0, EOL=10;
  while (c!=EOL && c!=EOF) c=fgetc(fid);
  return c;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief doesFileExist
//!
//! Check for file existence
//!
//////////////////////////////////////////////////////////////////////////////
int doesFileExist(const char *fname)
{
#ifdef isWin
  if(_access(fname, 0) == 0)
    return 0; // exists
  else
    return -1; // does not exist
#else // Linux
  if( access( fname, F_OK ) != -1 )
    return 0; // exists
  else
    return -1; // does not exist
#endif
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief doesFileExist
//!
//! Make a directory if it does not exist
//!
//////////////////////////////////////////////////////////////////////////////
int mkDirIfNec(const char *dirname)
{
   int retVal = 0;
#ifdef isWin
   if(!CreateDirectoryA(dirname ,NULL))
   {
      retVal = -1;
   }
#else
   if(mkdir(dirname, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1 )
   {
      retVal = -1;
   }
#endif

   return retVal;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief readChar
//!
//! read in a char array and store it in a std::string
//!
//////////////////////////////////////////////////////////////////////////////
std::string readChar(FILE *fid,int &readCountTotal)
{
   char tmpStr[80];
   std::string outStr;
   readCountTotal+=fscanf(fid,"%s",tmpStr);
   NextLine(fid);
   outStr=tmpStr;
   return(outStr);
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief readInt
//!
//! read in an int
//!
//////////////////////////////////////////////////////////////////////////////
int readInt(FILE *fid,int &readCountTotal)
{
   int tmpInt;
   readCountTotal+=fscanf(fid,"%d",&tmpInt);
   NextLine(fid);
   return(tmpInt);
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief readDouble
//!
//! read in a double
//!
//////////////////////////////////////////////////////////////////////////////
double readDouble(FILE *fid,int &readCountTotal)
{
   double tmpDouble;
   readCountTotal+=fscanf(fid,"%lf",&tmpDouble);
   NextLine(fid);
   return(tmpDouble);
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief readDoubleVector
//!
//! read in a vector of doubles
//!
//////////////////////////////////////////////////////////////////////////////
std::vector<double> readDoubleVector(FILE *fid,
                                     int &readCountTotal,
                                     const int vectorLen)
{

   std::vector<double> tmpVect(vectorLen);

   for(int i=0;i<vectorLen;i++)
   {
      readCountTotal += fscanf(fid,"%lf", &tmpVect[i]);
   }
   NextLine(fid);

   return tmpVect;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief readBool
//!
//! read in a bool
//!
//////////////////////////////////////////////////////////////////////////////
bool readBool(FILE *fid,int &readCountTotal)
{
   return(readInt(fid,readCountTotal)==1);
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief normalizeArcLength
//!
//! Normalize the monotonic vector s to lie in the range 0 and 1
//!
//////////////////////////////////////////////////////////////////////////////
int normalizeArcLength(std::vector<double> &s)
{
  s=(1.0/s[s.size()-1])*s;
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief smooth
//!
//! moving average filter for a vector of data
//!
//////////////////////////////////////////////////////////////////////////////
int smooth(std::vector<double>&x,int w)
{
   int n = (int)x.size();
   w = std::min(w,n);
   int wMid = w/2 + w%2 - 1;
   w = 2*wMid + 1;

   std::vector<double> x2(n);
   double xt, xte;

   x2[0] = x[0];
   x2[n-1] = x[n-1];

   for(int i=1; i<wMid; ++i)
   {
      xt = 0;
      xte = 0;
      int nPtsT = 2*i+1;
      for(int j=0; j<nPtsT; ++j)
      {
         xt += x[j];
         xte += x[n-j-1];
      }
      x2[i] = xt/nPtsT;
      x2[n-i-1] = xte/nPtsT;
   }
   for(int i=wMid; i<n-wMid; ++i)
   {
      xt=0;
      for(int j=i-wMid; j<i+wMid+1; ++j) xt += x[j];
      x2[i] = xt/w;
   }
   x = x2;
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief minsmooth
//!
//! minimum of a moving window for a vector
//!
//////////////////////////////////////////////////////////////////////////////
int minsmooth(std::vector<double>&x,int w)
{
   int n = (int)x.size();
   w = std::min(w,n);
   int wMid = w/2 + w%2 - 1;
   w = 2*wMid + 1;

   std::vector<double> x2(n);
   double xt, xte;

   x2[0] = x[0];
   x2[n-1] = x[n-1];

   for(int i=1; i<wMid; ++i)
   {
      xt = x[0];
      xte = x[n-1];
      int nPtsT = 2*i + 1;
      for(int j=1; j<nPtsT; ++j)
      {
         xt  = std::min(xt,x[j]);
         xte = std::min(xte,x[n-j-1]);
      }
      x2[i] = xt;
      x2[n-i-1] = xte;
   }
   for(int i=wMid; i<n-wMid; ++i)
   {
      xt = x[i-wMid];
      for(int j=i-wMid+1;j<i+wMid+1; ++j) xt = std::min(xt,x[j]);
      x2[i] = xt;
   }

   smooth(x2, w);
   for(int i=0; i<n; ++i) x[i] = std::min(x[i], x2[i]);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief decimate
//!
//! decimate a vector of data
//!
//////////////////////////////////////////////////////////////////////////////
int decimate(std::vector<double>&x,int w)
{
  int nIn=(int)x.size();
  int nOut=(nIn-1)/w+1;
  for(int i=0;i<nOut;i++) x[i]=x[w*i];
  if(w*(nOut-1)+1 != nIn) x[nOut-1]=x[nIn-1];

  x.resize(nOut);
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief solveQuadratic
//!
//! Solve the quadratic equation Ax^2 + Bx + C = 0 for x
//!
//////////////////////////////////////////////////////////////////////////////
int solveQuadratic(const double A,const double B,const double C,
  double &sol1,double &sol2)
{

  if(std::abs(A)<1e-308)
  {
    if(std::abs(B)<1e-308) return -2; // infinite solutions
    sol1=-C/B; sol2=sol1;
    return 0;
  }

  double rad,den,F1,F2;
  rad=B*B-4*A*C;
  if(rad<0) return -1;

  den=2*A;
  F1=-B/den; F2=std::sqrt(rad)/den;

  sol1=F1+F2;
  sol2=F1-F2;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief findMedian
//!
//! Find the median value for a vector of data
//!
//////////////////////////////////////////////////////////////////////////////
double findMedian(std::vector <double> &x)
{
  size_t nPts=x.size();
  std::vector<double> xSorted=x;
  std::sort (xSorted.begin(), xSorted.end());
  bool isOdd=nPts%2==1; // % is the MOD operator
  double xMED;
  size_t midPt=nPts/2;
  if(isOdd) xMED=xSorted[midPt];
  else      xMED=.5*(xSorted[midPt-1]+xSorted[midPt]);

  return(xMED);
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief LUsolve
//!
//! Solve linear system using LU decomposition or SVD
//!
//////////////////////////////////////////////////////////////////////////////
bool solveLinSys(const std::vector<std::vector<double>> &Av,
               const std::vector<double> &bv, std::vector<double> &xv,const bool isSVD)
{
  bool isIllCond=false;
  double minCondNum=100.0*std::numeric_limits<double>::epsilon();

  int i,j,dim=(int)bv.size();

  MatrixXd A(dim,dim);
  VectorXd b(dim),x(dim);

  for(i=0;i<dim;i++)
  {
    for(j=0;j<dim;j++) A(i,j)=Av[i][j];
    b[i]=bv[i];
  }

  if(isSVD)
  {
    JacobiSVD<MatrixXd> A_svd(A, ComputeThinU | ComputeThinV);
    double condNum = A_svd.singularValues()(0)
        / A_svd.singularValues()(A_svd.singularValues().size()-1);
    if(condNum<minCondNum) {isIllCond=true; return isIllCond;}
    x=A_svd.solve(b);
  }
  else x = A.lu().solve(b);

  for(i=0;i<dim;i++) xv[i]=x[i];
  return isIllCond;
}


//////////////////////////////////////////////////////////////////////////////
//!
//! \brief remClosePts
//!
//! remove close pts in input data
//!
//////////////////////////////////////////////////////////////////////////////
int remClosePts(std::vector<std::vector<double>> &x,
                    std::vector<std::vector<double>> &y,
                    double xThresh)
{
  printf("remClosePts():  ||dtheta||_min=%f deg imposed. ",xThresh);

  double dthetaThreshSQ=xThresh*xThresh;
  double dtheta,dthetaSQsum;
  bool anyRem=false;
  int i,j,curPt;
  int nPts=(int)x[0].size();
  int nPtsi=nPts;
  std::vector<double> isRem(nPts);

  int xLen=(int)x.size();
  int yLen=(int)y.size();

  // A minimum point-to-point distance threshold is imposed
  // At each iteration, only non-adjacent points can be designated for removal
  while(1)
  {
    anyRem=false;

    // Points are tagged for removal
    for(i=1;i<nPts;i++)
    {
      dthetaSQsum=0;
      for(j=0;j<xLen;j++)
      {
        dtheta=x[j][i]-x[j][i-1];
        dthetaSQsum+=dtheta*dtheta;
      }
      if(dthetaSQsum<dthetaThreshSQ && !isRem[i-1])
      {
        isRem[i]=true;
        anyRem=true;
      }
    }
    if(isRem[nPts-1] && nPts>2)
    {
      isRem[nPts-1]=false;
      isRem[nPts-2]=true;
      isRem[nPts-3]=false;
    }
    curPt=0;
    if(anyRem)
    {
      // Tagged points are removed from the trajectory
      for(i=0;i<nPts;i++)
      {
        if(!isRem[i])
        {
          for(j=0;j<xLen;j++) x[j][curPt]= x[j][i];
          if(yLen>0)
          {
             for(j=0;j<yLen;j++) y[j][curPt]= y[j][i];
          }
          curPt++;
        }
      }
      nPts=curPt;
      for(j=0;j<xLen;j++) x[j].resize(curPt);
      if(yLen>0)
      {
         for(j=0;j<yLen;j++) y[j].resize(curPt);
      }
      isRem.assign(nPts,false);
    }
    else break;
  }
  printf(" before %d points; after %d points\n",nPtsi,nPts);
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
//!
//! \brief aa2q
//!
//! axis-angle to quaternion orientation
//!
//////////////////////////////////////////////////////////////////////////////
std::array<double,4> aa2q(std::array<double,3> aa)
{
   std::array<double,4> q;
   double theta=std::sqrt(aa[0]*aa[0] + aa[1]*aa[1] + aa[2]*aa[2]);

   if(theta<1e-6)
   {
      q={1.0,0.0,0.0,0.0};
   }
   else
   {
      double sin_half_theta=std::sin(0.5*theta);
      q[0]=std::cos(0.5*theta);
      for(int i=0; i<3; i++)
      {
         q[i+1]=aa[i]*sin_half_theta/theta;
      }
   }
   return q;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief q2aa
//!
//! quaternion to axis-angle orientation
//!
//////////////////////////////////////////////////////////////////////////////
std::array<double,3> q2aa(std::array<double,4> q)
{
   std::array<double,3> aa;
   double norme=std::sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

   if(norme<1e-6)
   {
      aa={0.0,0.0,0.0};
   }
   else
   {
      double theta=2.0*std::atan2(norme,q[0])/norme;
      for(int i=0; i<3; i++)
      {
         aa[i]=theta*q[i+1];
      }
   }
   return aa;
}

