////////////////////////////////////////////////////////////////////////////
//! \file    robot.cpp
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief Robot-specific kinematics and dynamics
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
#include "util.h"
#include "robot.h"
#include "config.h"
#include <array>
#include <Eigen/Core> // for linear Algebra in fwdKinKuka()
#ifndef isWin
   #include <limits>
   #include "string.h"
#endif

using namespace Eigen;

namespace BATOTP{

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief set_robotType
//!
//! Set the robot type (define new robots here)
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::call_set_robotType(const std::string& robotTypeStr)
{
    return set_robotType(robotTypeStr);
}

int Robot::set_robotType(const std::string &robotTypeStr)
{
   _robotType=0;
   _robotTypeStr=robotTypeStr;
   if(_robotTypeStr=="KUKA")     _robotType=KUKA;
   if(_robotTypeStr=="UR")       _robotType=UR;
   if(_robotTypeStr=="RR")       _robotType=RR;
   if(_robotTypeStr=="CSPR3DOF") _robotType=CSPR3DOF;
   if(_robotTypeStr=="GENJNT")   _robotType=GENJNT;

   return(_robotType);
}

//! FORWARD KINEMATICS

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief fwdKin
//!
//! Forward displacement problem
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::call_fwdKin(const std::vector<std::vector<double> >& theta,
                       std::vector<std::vector<double>> &cart)
{return(fwdKin(theta,cart));}
int Robot::fwdKin(const std::vector<std::vector<double>> &theta,
                  std::vector<std::vector<double>> &cart)
{

   switch(_robotType)
   {
   case KUKA: //KUKA-LWR IV+
      fwdKinKuka(theta,cart);
      break;
   case RR: // planar RR positioning manipulator
      fwdKinRR(theta,cart);
      break;
   default:
      printf ("No forward Kinematics model provided for robotType=%s.\n",
              _robotTypeStr.c_str());
      return -1;
      assert(!"Unhandled case"); // Always assert unhandled cases
      break;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief fwdKinKuka
//!
//! Forward displacement problem for the Kuka LWR IV+
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::fwdKinKuka(const std::vector<std::vector<double> >& theta,
                      std::vector<std::vector<double>> &cart)
{
   int i;
   Matrix3d Q12,Q34,Q567,Q1234,Q;
   //op=[0; 0; .282];  .204+78 Robotiq hand with Robotiq fingers (center of pads)
   //op=[0; 0; .3145]; Robotiq hand with Robotiq fingers (tips of pads)
   Vector3d ToolPointVector(0,-.08,.545); // .468+.078 Robotiq hand with long fingers

   int nPts=(int)theta[0].size();

   double a0=.3105,a1=.4,a2=.39; // link lengths

   double t1,t2,t3,t4,t5,t6,t7;
   double c1,c2,c3,c4,c5,c6,c7;
   double s1,s2,s3,s4,s5,s6,s7;
   double x1,y1,z1;
   double x2,y2,z2;
   double x3,y3,z3;

   cart.resize(3);
   for(i=0;i<3;i++) cart[i].resize(nPts);

   for(i=0; i<nPts; i++)
   {
      t1=_DEG2RAD*theta[0][i]; c1=cos(t1); s1=sin(t1);
      t2=_DEG2RAD*theta[1][i]; c2=cos(t2); s2=sin(t2);
      t3=_DEG2RAD*theta[2][i]; c3=cos(t3); s3=sin(t3);
      t4=_DEG2RAD*theta[3][i]; c4=cos(t4); s4=sin(t4);
      t5=_DEG2RAD*theta[4][i]; c5=cos(t5); s5=sin(t5);
      t6=_DEG2RAD*theta[5][i]; c6=cos(t6); s6=sin(t6);
      t7=_DEG2RAD*theta[6][i]; c7=cos(t7); s7=sin(t7);

      Q12 <<  c1*c2, -s1, -c1*s2,
            c2*s1,  c1, -s1*s2,
            s2,   0,     c2;

      Q34 << c3*c4, -s3, c3*s4,
            c4*s3,  c3, s3*s4,
            -s4,   0,    c4;

      Q567 << c5*c6*c7 - s5*s7, - c7*s5 - c5*c6*s7, -c5*s6,
            c5*s7 + c6*c7*s5,   c5*c7 - c6*s5*s7, -s5*s6,
            c7*s6,             -s6*s7,     c6;

      Q1234=Q12*Q34; Q=Q1234*Q567;

      // elbow
      x1=a1*Q12(0,2);
      y1=a1*Q12(1,2);
      z1=a1*Q12(2,2)+a0;

      // wrist
      x2=x1+a2*Q1234(0,2);
      y2=y1+a2*Q1234(1,2);
      z2=z1+a2*Q1234(2,2);

      // tool point
      x3=x2+Q.row(0)*ToolPointVector;
      y3=y2+Q.row(1)*ToolPointVector;
      z3=z2+Q.row(2)*ToolPointVector;

      // normal (tool-pointing direction)
      //n << x3-x2, y3-y2, z3-z2 ;
      //n=n.normalized();

      cart[0][i]=x3;
      cart[1][i]=y3;
      cart[2][i]=z3;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief fwdKinRR
//!
//! Forward displacement problem for a RR planar robot
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::fwdKinRR(const std::vector<std::vector<double> >& theta,
                    std::vector<std::vector<double>> &cart)
{
   int i,nPts=(int)theta[0].size();
   double a1=.4,a2=.6;
   double th1,th2;
   cart.resize(3);
   for(i=0;i<3;i++) cart[i].resize(nPts);

   for(i=0;i<nPts;i++)
   {
      th1=_DEG2RAD*theta[0][i];
      th2=_DEG2RAD*theta[1][i];
      cart[0][i]=a1*std::cos(th1)+a2*std::cos(th1+th2);
      cart[1][i]=a1*std::sin(th1)+a2*std::sin(th1+th2);
   }
   return 0;
}

//! INVERSE KINEMATICS

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief invKin
//!
//! Inverse displacement problem
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::call_invKin(std::vector<std::vector<double>> &theta,
                       const std::vector<std::vector<double>> &cart)
{return(invKin(theta,cart));}
int Robot::invKin(std::vector<std::vector<double>> &theta,
                  const std::vector<std::vector<double>> &cart)
{
   switch(_robotType)
   {
   case CSPR3DOF:
      invKinCSPR3DOF(theta,cart);
      break;
   default:
      printf ("No inverse Kinematics model provided for robotType=%s.\n",
              _robotTypeStr.c_str());
      assert(!"Unhandled case"); // Always assert unhandled cases
      return -1;
      break;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief invKinCSPR3DOF
//!
//! Inverse kinematics for the large 3-DOF point-mass CSPR in PLT-00370:
//! finds cable lengths (theta) from Cartesian EE positions (cart) and
//! cable attacment points (pmat)
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::invKinCSPR3DOF(std::vector<std::vector<double>> &theta,
                          const std::vector<std::vector<double>> &cart)
{

   int i;
   int nDim=(int)cart.size();

   int nPts=(int)cart[0].size();
   theta.resize(3);

   for(i=0;i<nDim;i++) theta[i].resize(nPts);
   double x,y,z,rho1,rho2,rho3;
   std::array<double,3> rv1,rv2,rv3;

   if(_pmat.size()==0)
   {
      findCSPR3DOFpmat(_pmat);
   }

   for(i=0;i<nPts;i++)
   {
      x=cart[0][i];
      y=cart[1][i];
      z=cart[2][i];
      rv1={x-_pmat[0][0], y-_pmat[1][0], z-_pmat[2][0]};
      rv2={x-_pmat[0][1], y-_pmat[1][1], z-_pmat[2][1]};
      rv3={x-_pmat[0][2], y-_pmat[1][2], z-_pmat[2][2]};
      rho1=norm(rv1);
      rho2=norm(rv2);
      rho3=norm(rv3);
      theta[0][i]=rho1;
      theta[1][i]=rho2;
      theta[2][i]=rho3;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief findCSPR3DOFpmat
//!
//! Compute the cable attachment points for the large point-mass 3-DOF CSPR
//! in PLT-00370 at Laval University
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::findCSPR3DOFpmat(std::vector<std::vector<double>> &pmat)
{
   int i,j;
   std::array<double,3> cible1={1.0941, -4.9074, 2.5542};
   std::array<double,3> delta1={-0.765, 0.112, 3.74};
   std::array<double,3> cible3={0.2098, 5.3409, 2.6236};
   std::array<double,3> delta2={0.43, 0.125, 3.615};

   std::array<double,3> p1=cible1+delta1;
   std::array<double,3> p2=cible3+delta2;
   std::array<double,3> p3={-5.9751, 0.1399, 6.1543};

   pmat.resize(3); for(i=0;i<3;i++) pmat[i].resize(3);

   std::array<int,3> ind={1,0,2};
   for(i=0;i<3;i++)
   {
      int it=ind[i];
      pmat[i][0]=-p1[it];
      pmat[i][1]=-p2[it];
      pmat[i][2]=-p3[it];
   }

   std::array<double,3> centroide;
   for(i=0;i<3;i++) centroide[i]=1/3.0*(pmat[i][0]+pmat[i][1]+pmat[i][2]);

   for(i=0;i<3;i++)
   {
      for(j=0;j<3;j++) pmat[i][j]-=centroide[i];
   }
   return 0;
}

//! DYNAMICS

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief dynSerial
//!
//! Dynamics model for a serial robot
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::call_dynSerial(std::vector<std::vector<double>> &a1,
                   std::vector<std::vector<double>> &a2,
                   std::vector<std::vector<double>> &a3,
                   std::vector<std::vector<double>> &a4,
                   const std::vector<std::vector<double>> &theta,
                   const std::vector<std::vector<double>> &thetaD,
                   const std::vector<std::vector<double>> &thetaD2)
{return(dynSerial(a1,a2,a3,a4,theta,thetaD,thetaD2));}
int Robot::dynSerial(std::vector<std::vector<double>> &a1,
                     std::vector<std::vector<double>> &a2,
                     std::vector<std::vector<double>> &a3,
                     std::vector<std::vector<double>> &a4,
                     const std::vector<std::vector<double>> &theta,
                     const std::vector<std::vector<double>> &thetaD,
                     const std::vector<std::vector<double>> &thetaD2)
{
   switch(_robotType)
   {
   case RR:
      dynRR(a1,a2,a3,a4,theta,thetaD,thetaD2);
      break;
   default:
      printf ("No dynamics model provided for serial robotType=%s.\n",
              _robotTypeStr.c_str());
      assert(!"Unhandled case"); // Always assert unhandled cases
      return -1;
      break;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief dynRR
//!
//! Dynamics model for an RR planar robot
//!  - links are approximated as point masses
//!  - tau = a1*sddot + a2*sdot^2 + a3*sdot + a4
//!  - thetaD and thetaD2 are partial derivatives wrt s
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::dynRR(
      std::vector<std::vector<double>> &a1,
      std::vector<std::vector<double>> &a2,
      std::vector<std::vector<double>> &a3,
      std::vector<std::vector<double>> &a4,
      const std::vector<std::vector<double>> &theta,
      const std::vector<std::vector<double>> &thetaD,
      const std::vector<std::vector<double>> &thetaD2)
{
   int i;
   int nPts=(int)theta[0].size();
   double A1=.4,A2=.6,m1=4,m2=8;
   double th1,th2,dth1,dth2,ddth1,ddth2,c1,c2,c12,ccFact;
   double A11,A12,A22;
   for(i=0;i<2;i++)
   {
      a1[i].resize(nPts);
      a2[i].resize(nPts);
      a3[i].resize(nPts);
      a4[i].resize(nPts);
   }

   for(i=0;i<nPts;i++)
   {
      th1  =_DEG2RAD*theta[0][i];
      th2  =_DEG2RAD*theta[1][i];
      dth1 =_DEG2RAD*thetaD[0][i];
      dth2 =_DEG2RAD*thetaD[1][i];
      ddth1=_DEG2RAD*thetaD2[0][i];
      ddth2=_DEG2RAD*thetaD2[1][i];

      c1 =std::cos(th1);
      c2 =std::cos(th2);
      c12=std::cos(th1+th2);

      A11=.25*m1*A1*A1 + m2*(A1*A1 + .25*A2*A2 + A1*A2*c2);
      A12=.5*m2*(.5*A2*A2 + A1*A2*c2);
      A22=.25*m2*A2*A2;

      a1[0][i]=A11*dth1 + A12*dth2;
      a1[1][i]=A12*dth1 + A22*dth2;

      ccFact=m2*A1*A2*sin(th2);
      a2[0][i]=A11*ddth1 + A12*ddth2-ccFact*dth2*(dth1+.5*dth2);
      a2[1][i]=A12*ddth1 + A22*ddth2-.5*ccFact*dth1*dth1;

      //friction terms: (friction is zero if left unspecified)
      a3[0][i]=10*dth1;
      a3[1][i]=10*dth2;

      a4[0][i]=.5*_g*(m1*A1*c1 + m2*(2.0*A1*c1 + A2*c12));
      a4[1][i]=.5*_g*m2*A2*c12;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief dynParallel
//!
//! Dynamics model for a parallel robot
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
///
int Robot::call_dynParallel(std::vector<std::vector<double>> &a1,
                   std::vector<std::vector<double>> &a2,
                   std::vector<std::vector<double>> &a3,
                   std::vector<std::vector<double>> &a4,
                   const std::vector<std::vector<double>> &cart,
                   const std::vector<std::vector<double>> &cartD,
                   const std::vector<std::vector<double>> &cartD2)
{
    return dynParallel(a1,a2,a3,a4,cart,cartD,cartD2);
}

int Robot::dynParallel(std::vector<std::vector<double>> &a1,
                     std::vector<std::vector<double>> &a2,
                     std::vector<std::vector<double>> &a3,
                     std::vector<std::vector<double>> &a4,
                     const std::vector<std::vector<double>> &cart,
                     const std::vector<std::vector<double>> &cartD,
                     const std::vector<std::vector<double>> &cartD2)
{
   (void)cart;

   switch(_robotType)
   {
   case CSPR3DOF: //CSPR3DOF
      dynCSPR3DOF(a1,a2,a3,a4,cartD,cartD2);
      break;
   default:
      printf ("No dynamics model provided for parallel robotType=%s.\n",
              _robotTypeStr.c_str());
      assert(!"Unhandled case"); // Always assert unhandled cases
      return -1;
      break;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief dynCSPR3DOF
//!
//! Dynamics model for the large 3-DOF point-mass CSPR in PLT-00370
//!  - A*tau = a1*sddot + a2*sdot^2 + a3*sdot + a4
//!  - cartD and cartD2 are partial derivatives wrt s
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::dynCSPR3DOF(
      std::vector<std::vector<double>> &a1,
      std::vector<std::vector<double>> &a2,
      std::vector<std::vector<double>> &a3,
      std::vector<std::vector<double>> &a4,
      const std::vector<std::vector<double>> &cartD,
      const std::vector<std::vector<double>> &cartD2)
{
   int i,j;
   int nCart=3;
   int nPts =(int)cartD[0].size();

   for(i=0;i<nCart;i++)
   {
      a1[i].resize(nPts);
      a2[i].resize(nPts);
      a3[i].assign(nPts,0.0);
      a4[i].assign(nPts,0.0);
   }

   for(i=0;i<nPts;i++)
   {
      for(j=0;j<nCart;j++)
      {
         a1[j][i]=-cartD[j][i];
         a2[j][i]=-cartD2[j][i];
      }
      a4[2][i]=_g;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief setA
//!
//! Find the A matrix for the dynamics equation of a parallel robot
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
int Robot::call_setA(const std::vector<double> &theta,
         const std::vector<double> &cart,
         std::vector<std::vector<double>> &A)
{return(setA(theta,cart,A));}

int Robot::setA(const std::vector<double> &theta,
                const std::vector<double> &cart,
                std::vector<std::vector<double>> &A)
{
   switch(_robotType)
   {
   case CSPR3DOF:
      for(int i=0;i<3;i++)
      {
         for(int j=0;j<3;j++)
         {
            A[i][j]=(cart[i]-_pmat[i][j])/theta[j];
         }
      }
      break;

   default:
      printf ("isParallel=True and no code was provided to find the A matrix");
      printf("for robotType=%s.\n",_robotTypeStr.c_str());
      assert(!"Unhandled case"); // Always assert unhandled cases
      return -1;
      break;
   }
   return 0;
}

}
