////////////////////////////////////////////////////////////////////////////
//! \file    ba.cpp
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief   BA class functions
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
#include "ba.h"
#include "util.h"

#include <memory> // for shared ptrs
#ifndef isWin
#include <limits>
#endif
#include <cstring>
#include <numeric> //std::iota
#include <cassert>
#include <clocale>

namespace BATOTP{

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief BA
//!
//! Constructor
//!
//////////////////////////////////////////////////////////////////////////////
BA::BA(void)
{
   _A={1./5, 3./10, 4./5, 8./9, 1.0, 1.0};
   _dA=_A;
   for(int i=1; i<6; ++i)
   {
      _dA[i]=_A[i]-_A[i-1];
   }
   _dA[5]=1.0e-6;

   _E={71./57600, 0, -71./16695, 71./1920, -17253./339200, 22./525, -1./40};

   _B={1./5, 3./40,  44./45,  19372./6561,   9017./3168,    35./384,
          0, 9./40, -56./15, -25360./2187,     -355./33,          0,
          0,     0,   32./9,  64448./6561,  46732./5247,  500./1113,
          0,     0,       0,    -212./729,      49./176,    125./192,
          0,     0,       0,            0, -5103./18656, -2187./6784,
          0,     0,       0,            0,            0,      11./84};

   setErrorOptimization(NO_ERROR);

   #ifdef _WIN64
      _HomeFolder = "../../";
   #else
      _HomeFolder = "../";
   #endif

   _InputFolder  = _HomeFolder + "input/";
   _OutputFolder = _HomeFolder + "output/";
}
//////////////////////////////////////////////////////////////////////////////
//!
//! \brief BA
//!
//! Destructor
//!
//////////////////////////////////////////////////////////////////////////////
BA::~BA(void)
{
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief interpInputData
//!
//! Remove close pts in input data, perform kinematics computations, filter and
//! interpolate
//!
//////////////////////////////////////////////////////////////////////////////
int BA::interpInputData(Traj &traj)
{
   // remove close points and exit if traj. is too short
   if(traj.timestamp.size() > 0)
   {
      //verify that timestamps are in increasing order
      std::vector<uint8_t> isRem;
      isRem.reserve(traj.nPts);
      for(unsigned int i=1; i<traj.nPts; ++i)
      {
         if(traj.timestamp[i] == traj.timestamp[i-1])
         {
            isRem.push_back(i);
         }
      }
      for(int i=(int)isRem.size()-1; i>=0; --i)
      {
         traj.timestamp.erase(traj.timestamp.begin()+isRem[i]);
         for(unsigned int j=0; j<_nJoints; ++j)
         {
            traj.theta[j].erase(traj.theta[j].begin()+isRem[i]);
         }
         for(unsigned int j=0; j<_nCart; ++j)
         {
            traj.cart[j].erase(traj.cart[j].begin()+isRem[i]);
         }
      }
      isRem.clear();
      traj.nPts = (int)traj.timestamp.size();
      traj.tresInput = traj.timestamp.back()/(traj.nPts-1);
      traj.sres = traj.tresInput;
      traj.sC=traj.timestamp;
   }

   if(traj.nPts == 1)
   {
      printf("Input trajectory has less than one site initially so no optimization will be performed.\n");
      return -1;
   }
   if(traj.nPts < 4)
   {
      interpTrajLinear(traj,4);
   }

   if(_isInterpOnly)
   {
      traj.nPts = (int)traj.theta[0].size();
      double oldRes =traj.sres;
      traj.ptsOrig.resize(traj.nPts);

      std::iota(traj.ptsOrig.begin(), traj.ptsOrig.end(), 0);
      if( (_pathType==CART || _pathType==BOTH) && _nCart==6 )
      {
         aa2qVect(traj.cart);
      }

      evalSplineFullTraj(traj, oldRes, _outRes);

      if(_nCart==7)
      {
         q2aaVect(traj.cart);
      }
      traj.sres = _outRes;
      return -1;
   }

   traj.sLastSec=-1;
   int errorFlag;

   if(_pathType==CART)
   {
      remClosePts(traj.cart,traj.theta,_cartThresh);
      traj.nPts=(int)traj.cart[0].size();
   }
   else
   {
      remClosePts(traj.theta,traj.cart,_jntThresh);
      traj.nPts=(int)traj.theta[0].size();
   }

   if(traj.nPts == 1)
   {
      printf("Input trajectory has less than one site after remClosePts() so no optimization will be performed.\n");
      return -1;
   }
   if(traj.nPts < 4)
   {
      interpTrajLinear(traj,4);
   }

   if( (_pathType==CART || _pathType==BOTH) && _nCart==6 )
   {
      aa2qVect(traj.cart);
      traj.cartC.resize(_nCart);
      traj.cartpt.resize(_nCart);
      traj.cartDpt.resize(_nCart);
      traj.cartD2pt.resize(_nCart);
   }

   // decimate and smooth
   if(_inputDecimFact > 1)
   {
      if(_pathType==JOINT || _pathType==BOTH)
      {
         for(unsigned int i=0; i<_nJoints; i++)
         {
            smooth(traj.theta[i], _inputDecimFact);
         }
         for(unsigned int i=0; i<_nJoints; i++)
         {
            decimate(traj.theta[i], _inputDecimFact);
         }
         traj.nPts=(int)traj.theta[0].size();
      }
      if(_pathType==CART || _pathType==BOTH)
      {
         for(unsigned int i=0; i<_nCart; i++)
         {
            smooth(traj.cart[i],_inputDecimFact);
         }
         for(unsigned int i=0; i<_nCart; i++)
         {
            decimate(traj.cart[i],_inputDecimFact);
         }
         traj.nPts=(int)traj.cart[0].size();
      }
      traj.tresInput*=_inputDecimFact;
      traj.sres*=_inputDecimFact;
      _isInterpolated=true; // can't use timestamps after decimate
   }

   if(_smoothWindow >1 )
   {
      if(_pathType==JOINT || _pathType==BOTH)
      {
         for(unsigned int i=0; i<_nJoints; i++)
         {
            smooth(traj.theta[i],_inputDecimFact);
         }
      }
      if(_pathType==CART || _pathType==BOTH)
      {
         for(unsigned int i=0; i<_nCart; i++)
         {
            smooth(traj.cart[i],_inputDecimFact);
         }
      }
   }

   // forward kinematics
   if(_pathType==JOINT)
   {
      if( _isCartVelConOn || _isCartAccConOn)
      {
         errorFlag = myRobot.call_fwdKin(traj.theta,traj.cart);
         if(errorFlag==-1)
         {
            return -1;
         }
      }
      else
      {
         traj.cart.resize(_nCart);
         for(size_t i=0; i<_nCart; i++)
         {
            traj.cart[i].resize(traj.nPts);
         }
      }
   }

   // inverse kinematics
   if(_pathType==CART)
   {
      if (_isJntVelConOn || _isJntAccConOn || _isTrqConOn)
      {
         myRobot.call_invKin(traj.theta,traj.cart);
      }
      else
      {
         traj.theta.resize(3);
         for(unsigned int i=0; i<_nJoints; i++)
         {
            traj.theta[i].resize(traj.nPts);
         }
      }
   }

   traj.ptsOrig.resize(traj.nPts);
   std::iota(traj.ptsOrig.begin(), traj.ptsOrig.end(), 0);

   // interpolation to have constant s-resolution
   errorFlag=adjust_s(traj,"specialInterp");
   if(errorFlag==-1)
   {
      return -1;
   }

   errorFlag=adjust_s(traj,"regularInterp");
   if(errorFlag==-1)
   {
      return -1;
   }
   traj.sC.clear();

   evalSplineFullTraj(traj, traj.sres, traj.sres);
   traj.sdot.resize(traj.nPts, std::numeric_limits<double>::max());

   if(_isTrqConOn)
   {
      findDynModel(traj);
   }

   if(is_sdotOut)
   {
      traj.myMVChist.s.resize(4, std::vector<double>(0));
      traj.myMVChist.sdot.resize(4, std::vector<double>(0));
   }

   printf("Number of points on MVC, theta, and cart arrays after splineFact: %d\n",traj.nPts);

   return 0;
}


//////////////////////////////////////////////////////////////////////////////
//!
//! \brief aa2qVect
//!
//! axis-angle to quaternion for a vector of EE poses
//! each pose is (x,y,z,rx,ry,rz)
//!
//////////////////////////////////////////////////////////////////////////////
int BA::aa2qVect(std::vector<std::vector<double>> &pose)
{
   double qdir;
   std::array<double,3> aa;
   std::array<double,4> q,qprev;

   size_t nPts=pose[0].size();

   _nCart=7;
   pose.resize(_nCart);
   pose[6].resize(nPts);

   aa={pose[3][0],pose[4][0],pose[5][0]};
   qprev=aa2q(aa);

   for (size_t i=0; i<nPts; i++)
   {
      aa={pose[3][i],pose[4][i],pose[5][i]};
      q=aa2q(aa);

      qdir=0;
      for(int j=0; j<4; j++)
      {
         qdir+=q[j]*qprev[j];
      }
      if(qdir<0.0)
      {
         for(int j=0;j<4;j++)
         {
            q[j]=-q[j];
         }
      }
      qprev=q;

      for(int j=0;j<4;j++)
      {
         pose[3+j][i]=q[j];
      }
   }

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief q2aaVect
//!
//! quaternion to axis-angle
//! each pose is (x,y,z,q0,q1,q2,q3)
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
///
int BA::q2aaVect(std::vector<std::vector<double>> &pose)
{
   int i,j;
   std::array<double,3> aa;
   std::array<double,4> q;
   int nPts=(int)pose[0].size();

   for (i=0;i<nPts;i++)
   {
      q={pose[3][i],pose[4][i],pose[5][i],pose[6][i]};
      aa=q2aa(q);
      for(j=0;j<3;j++)
      {
         pose[3+j][i]=aa[j];
      }
   }

   _nCart=6;
   pose.resize(_nCart);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief adjust_s
//!
//! Adjust s according to the weights defined in the input file
//!
//////////////////////////////////////////////////////////////////////////////
int BA::adjust_s(Traj &traj,std::string interpType)
{
   // if s is the recorded time or the node positions, no adjustment is needed,
   // so we return:
   if(_sWeights[1]+_sWeights[2]<1e-8) return 0;

   double cartNormRes,thetaNormRes;
   if(interpType=="specialInterp")
   {
      cartNormRes=_cartNormRes;
      thetaNormRes=_thetaNormRes;
   }
   else
   {
      cartNormRes=_cartNormRes2;
      thetaNormRes=_thetaNormRes2;
   }

   int nPts=traj.nPts;
   int nPtsi=nPts;
   std::vector<double> thetaNorm(nPts),cartPosNorm(nPts);
   traj.sC.resize(nPts);
   traj.ptsOrig.resize(nPts);

   double dthetaSQ,dtheta_j,dcartPosSQ,dcartPos_j;
   double sLast,sResNew,tTeachFact,thetaNormFact,cartPosNormFact;
   double sResi=traj.sres;

   assert(_quadraticRadThresh > 0.0);
   double MinRatioDcartDtheta=1.0/_quadraticRadThresh;
   double thetaWindow=5; // degrees
   double thetaNormLast=0;
   double cartPosNormLast=0;

   if(!_areJointAnglesDegrees)
   {
      thetaWindow*=_DEG2RAD;
   }

   for(int i=0;i<nPts-1;i++)
   {
      dthetaSQ=0;
      for(unsigned j=0; j<_nJoints; j++)
      {
         dtheta_j=traj.theta[j][i+1]-traj.theta[j][i];
         dthetaSQ+=dtheta_j*dtheta_j;
      }
      double dtheta=std::sqrt(dthetaSQ);
      thetaNorm[i+1]=thetaNorm[i]+dtheta;

      dcartPosSQ=0;
      for(int j=0; j<3; j++)
      {
         dcartPos_j=traj.cart[j][i+1]-traj.cart[j][i];
         dcartPosSQ+=dcartPos_j*dcartPos_j;
      }
      double dcartPos=std::sqrt(dcartPosSQ);
      cartPosNorm[i+1]=cartPosNorm[i]+dcartPos;

      if(_isAutoIntegRes)
      {
         double thetaChange=thetaNorm[i+1]-thetaNormLast;
         double cartChange=cartPosNorm[i+1]-cartPosNormLast;
         if(thetaChange > thetaWindow)
         {
         MinRatioDcartDtheta=std::min(MinRatioDcartDtheta,3.0*cartChange/thetaChange);
            thetaNormLast=thetaNorm[i+1];
            cartPosNormLast=cartPosNorm[i+1];
         }
      }
   }

   if(thetaNorm[nPts-1] < thetaNormRes)
   {
      printf("Input trajectory points are all identical no optimization will be performed.\n");
      return -1;
   }

   sLast=0; sResNew=0;
   // For automatically setting _integRes based on the constraints and the
   // input trajectory:
   if(_isAutoIntegRes)
   {
      // switch theta-driven s 1 if Cart. position changes very little:
      if((cartPosNorm[nPts-1] < cartNormRes) && _scaleType==2)
      {
         _sWeights[1]=_sWeights[1]+_sWeights[2];
         _sWeights[2]=0;
         _scaleType=1;
      }

      // lower CartNormRes if joint pos. changes far exceed Cart. pos. changes:
      double sWeights12in = _sWeights[1]+_sWeights[2];
      double cartRat  = 500.0*cartPosNorm[nPts-1];
      double thetaRat = thetaNorm[nPts-1];
      if(!_areJointAnglesDegrees)
      {
         thetaRat*=_RAD2DEG;
      }

      double minIntegRes=0.004; // was 0.002
      double maxIntegRes=0.2;
      double KintegRes = 0.0003;
      double newIntegRes = KintegRes*_CartAccMax/_CartVelMax;
      for(unsigned int i=0; i<_nJoints; i++)
      {
        newIntegRes = std::max(newIntegRes, KintegRes*_JntAccMax[i]/_JntVelMax[i]);
      }
      newIntegRes = std::min(newIntegRes,maxIntegRes);

      double changeRat=cartRat/thetaRat;
      double jointIntegRes=maxIntegRes*changeRat*changeRat;

      double jointIntegResWindow=maxIntegRes*MinRatioDcartDtheta*MinRatioDcartDtheta;
      jointIntegResWindow=std::max(jointIntegResWindow,0.016);

      jointIntegRes=std::min(jointIntegRes,jointIntegResWindow);

      if(jointIntegRes<newIntegRes)
      {
         printf("InterpInputData(): Path-resolution integration resolution was changed from %f\n",newIntegRes);
         printf("                   to %f because orientation movement is dominant.\n",jointIntegRes);
         newIntegRes = jointIntegRes;
      }
      newIntegRes = std::max(newIntegRes,minIntegRes);

      printf("InterpInputData(): Final integ. res is %0.6f s.\n",newIntegRes);
      _integRes=newIntegRes;


      double sWeights12out=cartRat+thetaRat;
      double outScaleFact=sWeights12in/sWeights12out;
      cartRat*=outScaleFact;
      thetaRat*=outScaleFact;

      if(thetaRat>_sWeights[1])
      {
         _sWeights[1]=thetaRat;
         _sWeights[2]=cartRat;
      }
      if(_sWeights[2]>0)
      {
         cartNormRes=std::min(cartNormRes,cartNormRes*_sWeights[2]/_sWeights[1]);
      }
   }
   switch(_scaleType)
   {
   case 0: // scale s according to tTeach, the timestamp of the input traj.
      sLast=sResi*traj.ptsOrig[nPts-1];
      sResNew=sResi;
      break;
   case 1: // scale s according to dtheta
      sLast=thetaNorm[nPts-1];
      sResNew=thetaNormRes;
      break;
   case 2: // scale s according to dcart
      sLast=cartPosNorm[nPts-1];
      sResNew=cartNormRes;
      break;
   }
   printf("_sWeights: %f %f %f; ",_sWeights[0],_sWeights[1],_sWeights[2]);

   if(cartPosNorm[nPts-1] >= cartNormRes)
   {
      cartPosNormFact=_sWeights[2]*sLast/cartPosNorm[nPts-1];
   }
   else
   {
      cartPosNormFact=0;
   }
   tTeachFact =_sWeights[0]*sLast/(sResi*traj.ptsOrig[nPts-1]);
   thetaNormFact  =_sWeights[1]*sLast/thetaNorm[nPts-1];

   traj.sres=sLast/(nPts-1);
   for(int i=0; i<nPts; i++)
   {
      traj.sC[i]=tTeachFact*sResi*traj.ptsOrig[i]+thetaNormFact*thetaNorm[i]+\
            cartPosNormFact*cartPosNorm[i];
   }
   if(interpType=="specialInterp")
   {
      InterpVars myInterpVars;
      myInterpVars.tTeachFact=tTeachFact;
      myInterpVars.thetaNormFact=thetaNormFact;
      myInterpVars.cartPosNormFact=cartPosNormFact;
      myInterpVars.sLast=sLast;
      myInterpVars.sResNew=sResNew;
      myInterpVars.sResi=sResi;

      interpSpecial(traj, myInterpVars);
   }
   else
   {
      for(int i=1;i<nPts;i++)
      {
         if(traj.sC[i]-traj.sC[i-1] < 1e-12*traj.sres)
         {
            printf("adjust_s(): s-resolution is too small between two points. aborting... \n");
            return -1;
         }
      }
      evalSplineFullTraj(traj,traj.sres,sResNew);
   }

   if(_pathType==JOINT)
   {
      if(_robotType == GENJNT)
      {
         traj.cart.resize(_nCart);
         for(size_t i=0; i<_nCart; i++)
         {
            traj.cart[i].resize(traj.nPts);
         }
      }
      else
      {
         myRobot.call_fwdKin(traj.theta,traj.cart);
      }
   }

   if(_pathType==CART)  myRobot.call_invKin(traj.theta,traj.cart);

   printf("adjust_s() %s: number of trajpts: before %d; after %d\n",
          interpType.c_str(),nPtsi,traj.nPts);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief interpSpecial
//!
//! Interpolate using a special technique that verifies that each new point on
//! the interpolated curve is at the desired resolution from the previous point.
//! This avoids the problem with regular interpolation where noisy data can cause
//! large displacments along the old curve but small and higly variable displacements
//! along the new curve
//!
//////////////////////////////////////////////////////////////////////////////
int BA::interpSpecial(Traj &traj,const InterpVars &myInterpVars)
{
   double dthetaSQ,dtheta_j,dcartPos_j,dcartPosSQ;

   traj.thetaC.resize(_nJoints);
   for(unsigned int i=0; i<_nJoints; i++)
   {
      mySpline.getSplineCoeffs(traj.theta[i],traj.thetaC[i],"natural");
   }
   traj.cartC.resize(_nCart);
   for(unsigned int i=0; i<_nCart; i++)
   {
      mySpline.getSplineCoeffs(traj.cart[i],traj.cartC[i],"natural");
   }
   traj.curSegC=0; traj.tauC=0;
   int nPts2=(int)std::ceil(myInterpVars.sLast/myInterpVars.sResNew)+1;
   nPts2=std::max(nPts2,4);

   std::vector<double> sC2(nPts2);
   std::vector<std::vector<double>> theta(_nJoints,std::vector<double>(nPts2));
   std::vector<std::vector<double>> cart(_nCart,std::vector<double>(nPts2));
   for(unsigned int i=0; i<_nJoints; i++)
   {
      theta[i][0]=traj.theta[i][0];
   }
   for(unsigned int i=0; i<_nCart; i++)
   {
      cart[i][0]=traj.cart[i][0];
   }
   double sPrv=0,prv_ds=0;
   unsigned int CurNewPt=1, CurOldPt=1;

   int iter=0;
   bool isDone=false;

   while(!isDone)
   {
      iter++;
      dthetaSQ=0;
      for(unsigned int j=0; j<_nJoints; j++)
      {
         dtheta_j=traj.theta[j][CurOldPt]-theta[j][CurNewPt-1];
         dthetaSQ+=dtheta_j*dtheta_j;
      }

      dcartPosSQ=0;
      for(unsigned int j=0; j<3; j++)
      {
         dcartPos_j=traj.cart[j][CurOldPt]-cart[j][CurNewPt-1];
         dcartPosSQ+=dcartPos_j*dcartPos_j;
      }

      double cur_ds = myInterpVars.tTeachFact*myInterpVars.sResi*traj.ptsOrig[CurOldPt]+
            myInterpVars.thetaNormFact*std::sqrt(dthetaSQ)+
            myInterpVars.cartPosNormFact*std::sqrt(dcartPosSQ);

      if(cur_ds > myInterpVars.sResNew)
      {
         sC2[CurNewPt]=sPrv+myInterpVars.sResNew-prv_ds;
         prv_ds=0;
         sPrv=sC2[CurNewPt];
         traj.sCur=sPrv;
         if(traj.sCur>traj.sC[traj.nPts-1]) isDone=true;
         if(!isDone)
         {
            evalSplinePartials(traj); // don't need derivs - do this another way

            for(unsigned int i=0; i<_nJoints; i++)
            {
               theta[i][CurNewPt]=traj.thetapt[i];
            }
            for(unsigned i=0; i<_nCart; i++)
            {
               cart[i][CurNewPt]=traj.cartpt[i];
            }
            CurOldPt=traj.curSegC+1;

            CurNewPt++;
            if(CurNewPt == theta[0].size())
            {
               sC2.resize(CurNewPt+nPts2);
               for(unsigned int i=0; i<_nJoints; i++)
               {
                  theta[i].resize(CurNewPt+nPts2);
               }
               for(unsigned int i=0; i<_nCart; i++)
               {
                  cart[i].resize(CurNewPt+nPts2);
               }
            }
         }
      }
      else
      {
         if(CurOldPt == traj.nPts-1)
         {
            isDone=true;
         }
         else
         {
            prv_ds=cur_ds;
            sPrv=traj.sC[CurOldPt];
            CurOldPt++;
         }
      }
   }
   for(unsigned int i=0;i<_nJoints;i++)
   {
      theta[i][CurNewPt]=traj.theta[i][traj.nPts-1];
      theta[i].resize(CurNewPt+1);
   }
   for(unsigned int i=0;i<_nCart;i++)
   {
      cart[i][CurNewPt]=traj.cart[i][traj.nPts-1];
      cart[i].resize(CurNewPt+1);
   }
   traj.nPts=CurNewPt+1;
   traj.sres=myInterpVars.sResNew;
   traj.theta=theta;
   traj.cart=cart;

   if(traj.nPts<4) // interpolate to get a 4-point trajectory
   {
      interpTrajLinear(traj,4);
   }

   traj.ptsOrig.resize(traj.nPts);
   std::iota(traj.ptsOrig.begin(), traj.ptsOrig.end(), 0);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief evalSplineFullTraj
//!
//! find spline coefficients for the trajectory and interpolate at a new resolution
//!
//////////////////////////////////////////////////////////////////////////////
int BA::evalSplineFullTraj(Traj &myTraj,const double oldRes, double newRes)
{
   int nPtsOld,nPtsNew,errorFlag;

   nPtsOld=myTraj.nPts;
   myTraj.nPtsC=nPtsOld;
   nPtsNew=(int)std::ceil(oldRes/newRes*(nPtsOld-1))+1;
   nPtsNew=std::max(nPtsNew,4);
   newRes=oldRes*(nPtsOld-1)/(nPtsNew-1);

   if((int)myTraj.sC.size()!=nPtsOld) // if sC is empty
   {
      // input s
      myTraj.sC.resize(nPtsOld);
      std::iota(myTraj.sC.begin(), myTraj.sC.end(), 0);
      myTraj.sC=myTraj.sres*myTraj.sC;
   }

   // output s
   myTraj.sMVC.resize(nPtsNew);
   std::iota(myTraj.sMVC.begin(), myTraj.sMVC.end(), 0);

   double sScale=myTraj.sC[nPtsOld-1]/myTraj.sMVC[nPtsNew-1];
   myTraj.sMVC=sScale*myTraj.sMVC;

   myTraj.sresC=myTraj.sres;
   myTraj.vFact=1/myTraj.sresC;
   myTraj.aFact=myTraj.vFact*myTraj.vFact;
   myTraj.sres=newRes;
   myTraj.nPts=nPtsNew;

   // find spline coefficients
   myTraj.thetaC.resize(_nJoints);
   myTraj.cartC.resize(_nCart);
   for(unsigned int i=0;i<_nJoints;i++)
   {
      assert(myTraj.theta[i].size()>2);
      mySpline.getSplineCoeffs(myTraj.theta[i],myTraj.thetaC[i],"natural");
   }
   for(unsigned i=0; i<_nCart; i++)
   {
      mySpline.getSplineCoeffs(myTraj.cart[i] ,myTraj.cartC[i],"natural");
   }
   mySpline.getSplineCoeffs(myTraj.ptsOrig ,myTraj.ptsOrigC,"natural");

   // find spline segments and segment fractions (tau) for the sMVC locations
   Spline::splineSegs mySegs;
   errorFlag=mySpline.findInterpSegs(myTraj.sC,myTraj.sMVC,mySegs);
   if(errorFlag==-1) return -1;

   // Evaluate the spline+derivatives at the sMVC locations
   myTraj.thetaD.resize(_nJoints);
   myTraj.thetaD2.resize(_nJoints);
   for(unsigned i=0; i<_nJoints; i++)
   {
      mySpline.interp1spline(myTraj.theta[i],myTraj.thetaD[i],myTraj.thetaD2[i],
                             myTraj.thetaC[i],mySegs,oldRes);
   }

   myTraj.cartD.resize(_nCart);
   myTraj.cartD2.resize(_nCart);
   for(unsigned i=0; i<_nCart; i++)
   {
      mySpline.interp1spline(myTraj.cart[i],myTraj.cartD[i],myTraj.cartD2[i],
                             myTraj.cartC[i],mySegs,oldRes);
   }
   //if(flag) return -1;
   std::vector<double> dummy1,dummy2;
   mySpline.interp1spline(myTraj.ptsOrig, dummy1, dummy2,
                          myTraj.ptsOrigC, mySegs,oldRes);
    _isInterpolated=true;

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief findDynModel
//!
//! fill the vectors that describe the dynamic model for the robot with respect
//! to s
//!
//////////////////////////////////////////////////////////////////////////////
int BA::findDynModel(Traj &myTraj)
{

   if(_isParallelMech)
   {
      _dynDim=_nCart;
      myTraj.Apt.resize(_nCart);
      for(unsigned int i=0; i<_nCart; i++)
      {
         myTraj.Apt[i].resize(_nJoints);
      }
   }
   else
   {
      _dynDim=_nJoints;
   }

   myTraj.a1.resize(_dynDim);
   myTraj.a2.resize(_dynDim);
   myTraj.a3.resize(_dynDim);
   myTraj.a4.resize(_dynDim);

   myTraj.a1C.resize(_dynDim);
   myTraj.a2C.resize(_dynDim);
   myTraj.a3C.resize(_dynDim);
   myTraj.a4C.resize(_dynDim);

   myTraj.a1pt.resize(_dynDim);
   myTraj.a2pt.resize(_dynDim);
   myTraj.a3pt.resize(_dynDim);
   myTraj.a4pt.resize(_dynDim);

   if(_isParallelMechOrig)
   {
      myRobot.call_dynParallel(myTraj.a1,myTraj.a2,myTraj.a3,myTraj.a4,
                               myTraj.cart,myTraj.cartD,myTraj.cartD2);
   }
   else
   {
      myRobot.call_dynSerial(myTraj.a1,myTraj.a2,myTraj.a3,myTraj.a4,
                             myTraj.theta,myTraj.thetaD,myTraj.thetaD2);
   }

   if (_isParallelMech && _isPar2Ser)
   {
      std::vector<double> cartpt(_nCart),thetapt(_nJoints);

      for(unsigned int i=0; i<myTraj.nPts; i++)
      {
         for(unsigned int j=0; j<_nCart; j++)
         {
            cartpt[j]=myTraj.cart[j][i];
         }
         for(unsigned int j=0; j<_nJoints; j++)
         {
            thetapt[j]=myTraj.theta[j][i];
         }
         myRobot.call_setA(thetapt,cartpt,myTraj.Apt);

         dynCoeffs2Ser(myTraj.Apt,myTraj.a1,_isSVD,i);
         dynCoeffs2Ser(myTraj.Apt,myTraj.a2,_isSVD,i);
         dynCoeffs2Ser(myTraj.Apt,myTraj.a3,_isSVD,i);
         dynCoeffs2Ser(myTraj.Apt,myTraj.a4,_isSVD,i);
      }
      _isParallelMech=false;
   }

   for(unsigned int i=0; i<_nJoints; i++)
   {
      mySpline.getSplineCoeffs(myTraj.a1[i],myTraj.a1C[i],"natural");
      mySpline.getSplineCoeffs(myTraj.a2[i],myTraj.a2C[i],"natural");
      mySpline.getSplineCoeffs(myTraj.a3[i],myTraj.a3C[i],"natural");
      mySpline.getSplineCoeffs(myTraj.a4[i],myTraj.a4C[i],"natural");
   }

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief dynCoeffs2Ser
//!
//! Convert one term of the RHS of the dynamic model to serial-robot form
//!
//////////////////////////////////////////////////////////////////////////////
int BA::dynCoeffs2Ser(std::vector<std::vector<double>> &A,
                      std::vector<std::vector<double>> &b,bool isSVD,int trajPt)
{
   int j,nCart=(int)b.size();
   std::vector<double> bStar(nCart),xStar(nCart);
   for(j=0;j<nCart;j++) bStar[j]=b[j][trajPt];
   solveLinSys(A,bStar,xStar,isSVD);
   for(j=0;j<nCart;j++) b[j][trajPt]=xStar[j];
   return 0;
}

// *********** ACCEL. CURVE INTEGRATION FUNCTIONS ************

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief sweep
//!
//! fwd and rev integration sweeping, imposing second-order constraints
//!
//////////////////////////////////////////////////////////////////////////////

int BA::sweep(Traj &traj)
{
   int nIter=0;
   int nFailed=0;
   int nODEeval=0;
   const int maxIntegSteps=(int)std::floor(_maxIntegTime/_integRes)+1;
   const int nChunk=10000;
   int nIntegPts=nChunk;

   double h,sLast;
   double absh=_integRes;
   double tElapsed=0;

   std::vector<double> sInteg(nIntegPts),tInteg(nIntegPts),sdotInteg(nIntegPts);
   std::array<double,7> sArr, sdotArr, sddotArr;
   sArr.fill(0);
   sdotArr.fill(0);
   sddotArr.fill(0);

   mySpline.getSplineCoeffs(traj.sdot ,traj.sdotC,"natural");

   if(_integDir==1)
   {
      traj.curSegC=0;
      traj.tauC=0;
      sArr[0]=0;

      traj.curSegMVC=0;
      traj.tauMVC=0;
      sLast=traj.sC[traj.nPtsC-1];
   }
   else
   {
      traj.curSegC=traj.nPtsC-2;
      traj.tauC=1;
      sArr[0] = traj.sC[traj.nPtsC-1];

      traj.curSegMVC=traj.nPts-2;
      traj.tauMVC=1;
      sLast=0;
   }

   traj.sCur = sArr[0];
   traj.sdotCur = 0;

   applyAccelConstraintsBisectionPt(traj, sddotArr[0], nIter);

   h=_integDir*absh;
   sdotArr[0]=.1 * h * sddotArr[0]; // was .01

   // Velocity constraints for the first point of the traj.
   // This prevents the problem of low speeds and high integration time steps
   _sdotMin=sdotArr[0]; //this should replace the code below

   sdotLim(traj,sdotArr[0],"linear");
   _sdotMin=sdotArr[0];
   traj.sdotCur=sdotArr[0];

   sInteg[0]=sArr[0];
   applyAccelConstraintsBisectionPt(traj, sddotArr[0], nIter);
   sdotArr[0] = traj.sdotCur;
   sdotLim(traj, sdotArr[0], "linear");
   sdotInteg[0] = sdotArr[0];

   sInteg.resize(nIntegPts);
   sdotInteg.resize(nIntegPts);
   double sdotT,sddotT;

   h=_integDir*absh;
   int nPts=0;

   double dsMin=1.0e-6*sArr.back()/sArr.size();
   std::array<double,6> dsMinV=dsMin*_dA;

   for(int i=1; i<nIntegPts; i++) // the main loop
   {
      double s0=traj.sCur;
      traj.sdotLimTypeT=false;
      sdotT = sdotArr[0];
      sddotT = sddotArr[0];
      sArr[6]   =sArr[0]   +h*sdotT;
      sdotArr[6]=sdotArr[0]+h*sddotT;

      traj.sCur=sArr[6];
      sdotLim(traj, sdotArr[6], "linear");

      traj.sCur=s0;
      int nMVC_LimSubSteps=0;

      for(int j=0; j<6; j++)
      {
         traj.sdotLimTypeT=false;
         sdotT=0;
         sddotT=0;
         for(int k=0; k<j+1; k++)
         {
            sdotT +=_B[k][j]*sdotArr[k];
            sddotT+=_B[k][j]*sddotArr[k];
         }

         sArr[j+1]   =sArr[0]   + h*sdotT;
         sdotArr[j+1]=sdotArr[0]+ h*sddotT;

         // force change in s to be monotonic
         //double dsArr=_integDir*std::max(dsMinV[j],_integDir*(sArr[j+1]-sArr[j]));
         //sArr[j+1] = sArr[j] + dsArr; //can cause discontinuities in the output traj.
         sdotArr[j+1]=std::max(sdotArr[j+1],dsMinV[j]/absh); //don't need _sdotMin with this?

         traj.sCur=sArr[j+1];

         sdotLim(traj, sdotArr[j+1], "linear");
         traj.sdotCur=sdotArr[j+1];
         applyAccelConstraintsBisectionPt(traj, sddotArr[j+1], nIter);
         sdotArr[j+1]=traj.sdotCur;
         if(traj.sdotLimTypeT) nMVC_LimSubSteps++;
      }

      sArr[0]    =sArr[6];
      sdotArr[0] =sdotArr[6];
      sddotArr[0]=sddotArr[6];
      sInteg[i]   =sArr[0];
      sdotInteg[i]=sdotArr[0];

      if(i==nIntegPts-1)
      {
         nIntegPts+=nChunk;
         sInteg.resize(nIntegPts);
         sdotInteg.resize(nIntegPts);
      }

      if(traj.sCur*_integDir>sLast) // integration has completed
      {
         nODEeval=4*i;
         tElapsed=absh*i;
         nPts=i + 1;
         break;
      }

      if(i>maxIntegSteps)
      {
         printf("Error in sweep(): maxIntegTime of %.1f s was exceeded.\n",_maxIntegTime);
         setErrorOptimization(MAX_INTEGRATION_TIME);
         return -1;
      }
   }

   // resize the output arrays to have exactly the number of integrated points
   sInteg.resize(nPts);
   sdotInteg.resize(nPts);
   tInteg.resize(nPts);

   // interpolate linearly so that last s-value of the integrated curve is the same as
   // it was for the input s-sdot curve
   double sRat=(sLast-sInteg[nPts-2])/(sInteg[nPts-1]-sInteg[nPts-2]);
   sdotInteg[nPts-1]=sdotInteg[nPts-2]+sRat*(sdotInteg[nPts-1]-sdotInteg[nPts-2]);
   sInteg[nPts-1]=sLast;

   if (_integDir==1)
   {
      printf("fwd. integ.: %4d steps; %5d ODE evals; %3d failed steps; traj time. %.3f sec.; avg. step size %f sec.\n",\
             nPts,nODEeval,nFailed,tElapsed,tElapsed/nPts);
      sdotInteg[nPts-1]=traj.sdot[traj.nPts-1];
   }
   else
   {
      // reverse the integrated curve
      std::reverse(sInteg.begin(),sInteg.end());
      std::reverse(sdotInteg.begin(),sdotInteg.end());

      // the final sdot value obtained on the trajectory via reverse integration is replaced
      // by the initial value of sdot for the forward integration

      printf("rev. integ.: %4d steps; %5d ODE evals; %3d failed steps; traj time. %.3f sec.; avg. step size %f sec.\n",\
             nPts,nODEeval,nFailed,tElapsed,tElapsed/nPts);
   }
   std::iota(tInteg.begin(), tInteg.end(), 0);
   tInteg=absh*tInteg;
   traj.tTotalTraj=tElapsed;
   if(is_sdotOut ) // store the integrated s-sdot curve for file ouput
   {
      if(_isLastSweep)
      {
         traj.myMVChist.s[1]=sInteg;
         traj.myMVChist.sdot[1]=sdotInteg;
      }
      else
      {
         traj.myMVChist.s[0]=sInteg;
         traj.myMVChist.sdot[0]=sdotInteg;
      }
   }

   if(nPts < 4)
   {
      double tResNew=tInteg[nPts-1]/3.;
      nPts=4;
      std::vector<double> tIntegNew(nPts);
      std::iota(tIntegNew.begin(), tIntegNew.end(), 0);
      tIntegNew=tResNew*tIntegNew;

      Spline::splineSegs mySegs;
      mySpline.findInterpSegs(tInteg,tIntegNew,mySegs);
      mySpline.interp1linear(sInteg,mySegs);
      mySpline.interp1linear(sdotInteg,mySegs);
      tInteg = tIntegNew;
   }

   // copy integrated MVC to myTraj (variable s-resolution)
   if(_isLastSweep) traj.tMVC=tInteg;
   traj.sMVC=sInteg;
   traj.sdot=sdotInteg;
   traj.nPts=nPts;

   assert(nPts>2);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief sdotLim
//!
//! restrict sdot using the MVC and a velocity constraints
//!
//////////////////////////////////////////////////////////////////////////////
int BA::sdotLim(Traj &myTraj,double &sdot,const std::string &type)
{
   double sdoti=sdot;
   if(_integDir==1){
      double sdotMVC=evalsdot(myTraj,type);
      if(sdot>sdotMVC)
      {
         myTraj.isOn_sdot=true;
         sdot=sdotMVC;
      }
      else myTraj.isOn_sdot=false;
   }
   sdot=std::min(sdot,myTraj.sC.back()/_integRes);
   sdot=std::max(sdot,_sdotMin);
   // Joint velocity constraints
   for(unsigned int i=0;i<_nJoints;i++)
   {
      if(std::abs(myTraj.thetaDpt[i]) > _jntThresh*myTraj.vFact)
      {
         sdot=std::min(sdot,std::abs(_JntVelMax[i]/myTraj.thetaDpt[i]));
      }
   }
   if(_isCartVelConOn && myTraj.CartAccCoeffs[0] > _quadraticRadThresh*myTraj.aFact)
   {
      sdot=std::min(sdot,_CartVelMax/std::sqrt(myTraj.CartAccCoeffs[0]));
   }
   if(sdot<sdoti)
   {
      myTraj.sdotLimTypeT=true;
   }

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief applyAccelConstrantsBisectionPt
//!
//! Apply all second-order constraints to a single trajectory point
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
int BA::applyAccelConstraintsBisectionPt(Traj &traj, double &sddot, int &nIter)
{
   double sdotErr,sdotGoodLast;
   const double sdotErrThresh=.001; // convergence tolerance
   double lowFact=.01;
   double sdotMin=0;
   double sdotGood=sdotMin;
   bool isViol;
   bool anyGoodIter=false;
   double sddotmax=2*traj.sC.back()/(_integRes*_integRes);

   double sdotL=sdotGood;
   double sdotH=traj.sdotCur;
   double sdotCur=sdotH;
   nIter=0;
   int nViol=0;

   evalSplinePartials(traj);

   // A Bisection search algorithm is used to find the max. value of sdot that respects
   // all accel. constraints. This is more robust but less efficient than applyAccelConstraintsDirect(),
   // which explicitly solves for sdot in each pair of acceleration constraint equations
   while(1)
   {
      isViol=verifySecondOrderConstraints(traj, sdotCur, sddotmax);
      if(isViol)
      {
         if (_integDir==-1 && traj.sLastSec<0)
         {
            traj.sLastSec=traj.sCur;
         }
         nViol++;
         sdotH=sdotCur;
         if(!anyGoodIter)
         {
            lowFact*=2.0;
            sdotL=std::max(.999*sdotMin,(1.0-lowFact)*sdotH);
         }
      }
      else
      {
         if(nIter==0)
         {
            break;
         }
         nViol=0;
         anyGoodIter=true;
         sdotGoodLast=sdotGood;
         sdotGood=sdotCur;
         sdotErr=std::abs(sdotGood-sdotGoodLast)/sdotGood;
         if(sdotErr<sdotErrThresh || sdotCur<sdotMin)
         {
            traj.sdotCur=sdotCur;
            break;
         }
         sdotL=sdotCur;
      }
      nIter++;

      if(nIter>100)
      {
         printf("\napplyAccelConstraintsBisectionPt() error:\n");
         printf("  sdot point not found after %d Bisection iterations\n",nIter);
         return -1;
      }
      if(sdotCur<0 || ((sdotH-sdotL)/sdotH<1e-20 && !anyGoodIter))
      {
         printf("applyAccelConstraintsBisectionPt() error:\n");
         printf("  sdot was reduced to %f and\n",sdotCur);
         printf("  did not respect accel constraints.\n");
         return -1;
      }
      sdotCur=.5*(sdotH+sdotL);
   }

   if(_integDir==1)
   {
      sddot=traj.sddotH;
   }
   else
   {
      sddot=traj.sddotL;
   }
   return 0;
}
//////////////////////////////////////////////////////////////////////////////
//!
//! \brief evalSplinePartials
//!
//! evaluate first- and second-order partial derivatives of joint angle positions
//! with respect to s using spline interpolants
//!
//////////////////////////////////////////////////////////////////////////////
int BA::evalSplinePartials(Traj &traj)
{
   double c0,c1,c2,c3;

   updateCurSeg(traj.sC, traj.sCur, traj.curSegC, traj.tauC);

   double tau=traj.tauC;
   double tau2=tau*tau;
   double tau3=tau2*tau;

   for(size_t i=0; i<_nJoints; i++)
   {
      c0=traj.thetaC[i].c0[traj.curSegC];
      c1=traj.thetaC[i].c1[traj.curSegC];
      c2=traj.thetaC[i].c2[traj.curSegC];
      c3=traj.thetaC[i].c3[traj.curSegC];

      traj.thetapt[i] =    c3*tau3 +   c2*tau2 + c1*tau +c0;
      traj.thetaDpt[i] =(3*c3*tau2 + 2*c2*tau  + c1)*traj.vFact;
      traj.thetaD2pt[i]=(6*c3*tau  + 2*c2)*traj.aFact;
   }
   if(_isCartVelConOn || _isCartAccConOn){
      assert(_nCart <= traj.cartC.size());
      assert(_nCart <= traj.cartpt.size());
      assert(_nCart <= traj.cartDpt.size());
      assert(_nCart <= traj.cartD2pt.size());

      for(size_t i=0; i<_nCart; i++)
      {
         c0=traj.cartC[i].c0[traj.curSegC];
         c1=traj.cartC[i].c1[traj.curSegC];
         c2=traj.cartC[i].c2[traj.curSegC];
         c3=traj.cartC[i].c3[traj.curSegC];

         traj.cartpt[i] =    c3*tau3 +   c2*tau2 + c1*tau +c0;
         traj.cartDpt[i] =(3*c3*tau2 + 2*c2*tau  + c1)*traj.vFact;
         traj.cartD2pt[i]=(6*c3*tau  + 2*c2)*traj.aFact;
      }
      evalCartQuadCoeffs(traj);
   }

   if(_isTrqConOn && traj.a1C.size()>0)
   {
      double tau3=tau2*tau;
      for(size_t i=0; i<_nJoints; i++)
      {
         traj.a1pt[i] = traj.a1C[i].c3[traj.curSegC]*tau3 +
                        traj.a1C[i].c2[traj.curSegC]*tau2 +
                        traj.a1C[i].c1[traj.curSegC]*tau  +
                        traj.a1C[i].c0[traj.curSegC];

         traj.a2pt[i]=traj.a2C[i].c3[traj.curSegC]*tau3 +
               traj.a2C[i].c2[traj.curSegC]*tau2 +
               traj.a2C[i].c1[traj.curSegC]*tau  +
               traj.a2C[i].c0[traj.curSegC];

         traj.a3pt[i]=traj.a3C[i].c3[traj.curSegC]*tau3 +
               traj.a3C[i].c2[traj.curSegC]*tau2 +
               traj.a3C[i].c1[traj.curSegC]*tau  +
               traj.a3C[i].c0[traj.curSegC];

         traj.a4pt[i]=traj.a4C[i].c3[traj.curSegC]*tau3 +
               traj.a4C[i].c2[traj.curSegC]*tau2 +
               traj.a4C[i].c1[traj.curSegC]*tau  +
               traj.a4C[i].c0[traj.curSegC];
      }
      if(_isParallelMech)
      {
         myRobot.call_setA(traj.thetapt,traj.cartpt,traj.Apt);
      }
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief evalCartQuadCoeffs
//!
//! Evaluate coefficients of the quadratic equation for the Cartesian acceleration
//! constraint
//!
//////////////////////////////////////////////////////////////////////////////
int BA::evalCartQuadCoeffs(Traj &myTraj)
{
   double vx,vy,vz,ax,ay,az; // partials wrt s

   vx=myTraj.cartDpt[0];
   vy=myTraj.cartDpt[1];
   vz=myTraj.cartDpt[2];
   ax=myTraj.cartD2pt[0];
   ay=myTraj.cartD2pt[1];
   az=myTraj.cartD2pt[2];

   myTraj.CartAccCoeffs[0] =   vx*vx + vy*vy + vz*vz;
   myTraj.CartAccCoeffs[1] = 2*(vx*ax + vy*ay + vz*az);
   myTraj.CartAccCoeffs[2] =    ax*ax + ay*ay + az*az;

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief verifySecondOrderConstraints
//!
//! Verify if second order constraints are respected for the current value of
//! sdot
//!
//////////////////////////////////////////////////////////////////////////////
bool BA::verifySecondOrderConstraints(Traj &myTraj,const double sdotCur, \
                                      const double sddotMax)
{
   myTraj.sddotL=-sddotMax;
   myTraj.sddotH= sddotMax;

   double sdotSQ=sdotCur*sdotCur;
   double CartAccMaxSQ=_CartAccMax*_CartAccMax;
   double vpt,a1pt,vTerm,sddotCmax,sddotCmin;
   double A,B,C,sol1,sol2;
   int svpt,errorFlag;

   if(_isTrqConOn)
   {
      if(_isParallelMech)
      {
         std::vector<double> cStar1(_nCart);
         std::vector<double> bStar(_nCart),xStar(_nCart);
         std::vector<std::vector<double>> Astar;
         std::array<double,2> sddotSol,trqLim;

         for(size_t i=0; i<_nCart; i++)
         {
            cStar1[i]=sdotSQ*myTraj.a2pt[i]+sdotCur*myTraj.a3pt[i]+myTraj.a4pt[i];
         }
         for(size_t j=0; j<_nJoints; j++)
         {
            trqLim={_JntTrqMin[j],_JntTrqMax[j]};
            for(size_t ii=0; ii<2; ii++)
            {
               Astar=myTraj.Apt;
               for(size_t k=0; k<_nCart; k++)
               {
                  bStar[k]=cStar1[k]-myTraj.Apt[k][j]*trqLim[ii];
                  Astar[k][j]=-myTraj.a1pt[k];
               }
               solveLinSys(Astar,bStar,xStar,_isSVD);
               sddotSol[ii]=xStar[j];
            }
            myTraj.sddotH=std::min(myTraj.sddotH,std::max(sddotSol[0],sddotSol[1]));
            myTraj.sddotL=std::max(myTraj.sddotL,std::min(sddotSol[0],sddotSol[1]));
            if(myTraj.sddotL>myTraj.sddotH) return true;
         }
      }
      else
      {
         for(size_t j=0; j<_nJoints; j++)
         {
            a1pt=myTraj.a1pt[j];
            double tmp1=myTraj.a3pt[j]*sdotCur+myTraj.a4pt[j];

            if(std::abs(a1pt)<_jntThresh*myTraj.vFact) continue;
            double tmp2=myTraj.a2pt[j]*sdotSQ+tmp1;

            std::array<double,2> sddotSol;
            sddotSol[0]=(_JntTrqMax[j]-tmp2)/a1pt;
            sddotSol[1]=(_JntTrqMin[j]-tmp2)/a1pt;
            myTraj.sddotH=std::min(myTraj.sddotH,std::max(sddotSol[0],sddotSol[1]));
            myTraj.sddotL=std::max(myTraj.sddotL,std::min(sddotSol[0],sddotSol[1]));
            if(myTraj.sddotL>myTraj.sddotH) return true;
         }
      }
   }

   // Joint accel. constraints
   if(_isJntAccConOn)
   {
      for(size_t j=0;j<_nJoints;j++)
      {
         vpt=myTraj.thetaDpt[j];
         if(std::abs(vpt)<_jntThresh*myTraj.vFact)
         {
            if(std::abs(myTraj.thetaD2pt[j])<_jntThresh*myTraj.aFact) continue;
            if(sdotSQ>_JntAccMax[j]/std::abs(myTraj.thetaD2pt[j])) return true;
            else continue;
         }

         svpt=sgn(vpt);
         vTerm=myTraj.thetaD2pt[j]*sdotSQ;

         myTraj.sddotH=std::min(myTraj.sddotH,( svpt*_JntAccMax[j]-vTerm)/vpt);
         myTraj.sddotL=std::max(myTraj.sddotL,(-svpt*_JntAccMax[j]-vTerm)/vpt);

         if(myTraj.sddotL>myTraj.sddotH) return true;
      }
   }
   if(_isCartAccConOn)
   {
      // Cart. accel. constraint
      A=myTraj.CartAccCoeffs[0];
      \
      if(A>_quadraticRadThresh*myTraj.aFact)
      {
         B = myTraj.CartAccCoeffs[1]*sdotSQ;
         C = myTraj.CartAccCoeffs[2]*sdotSQ*sdotSQ-CartAccMaxSQ;

         errorFlag=solveQuadratic(A,B,C,sol1,sol2);

         if(errorFlag==-1)
         {
            return true;
         }

         sddotCmax=std::max(sol1,sol2);
         sddotCmin=std::min(sol1,sol2);

         myTraj.sddotH=std::min(myTraj.sddotH,sddotCmax);
         myTraj.sddotL=std::max(myTraj.sddotL,sddotCmin);
         if(myTraj.sddotL>myTraj.sddotH)
         {
            return true;
         }
      }
      else
      {
         C=myTraj.CartAccCoeffs[2];
         if(C<_quadraticRadThresh*_quadraticRadThresh*myTraj.aFact*myTraj.aFact)
         {
            return false;
         }

         if(sdotSQ*sdotSQ>CartAccMaxSQ/C)
         {
            return true;
         }
         else
         {
            return false;
         }
      }
   }
   return false;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief evalsdot
//!
//! Evaluate the MVC at myTraj.sCur using linear interpolation
//!
//////////////////////////////////////////////////////////////////////////////
double BA::evalsdot(Traj &myTraj,const std::string &type)
{
   updateCurSeg(myTraj.sMVC,myTraj.sCur,myTraj.curSegMVC,myTraj.tauMVC);
   int seg=myTraj.curSegMVC;
   double sdotMVC;

   if(type=="linear")
      sdotMVC=myTraj.sdot[seg]+myTraj.tauMVC*(myTraj.sdot[seg+1]-myTraj.sdot[seg]);
   else
   {
      double tau=myTraj.tauMVC;
      double tau2=tau*tau;
      double tau3=tau2*tau2;
      sdotMVC=myTraj.sdotC.c3[seg]*tau3 + myTraj.sdotC.c2[seg]*tau2 + \
            myTraj.sdotC.c1[seg]*tau  + myTraj.sdotC.c0[seg];
   }
   return(std::max(sdotMVC,_sdotMin));
}


//////////////////////////////////////////////////////////////////////////////
//!
//! \brief updateCurSeg
//!
//! find the segment of the spline with sites s that contains the value sCur
//!
//////////////////////////////////////////////////////////////////////////////
int BA::updateCurSeg(const std::vector<double> &s, const double sCur,
                     int &curSeg,double &tau)
{
   double sSeg;
   const int lastSeg = (int)s.size()-2;

   while(1)
   {
      sSeg=s[curSeg];

      if(sCur >= sSeg && sCur <= s[curSeg+1])
      {
         break;
      }
      if(sCur > sSeg)
      {
         if(curSeg >= lastSeg)
         {
            curSeg = lastSeg;
            break;
         }
         curSeg++;
      }
      if(sCur < sSeg)
      {
         if(curSeg <= 0)
         {
            curSeg = 0;
            break;
         }
         curSeg--;
      }
   }
   tau=(sCur-sSeg)/(s[curSeg+1]-sSeg);
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief interpOutputData
//!
//! Interpolate the optimized trajectory to the output resolution
//!
//////////////////////////////////////////////////////////////////////////////
int BA::interpOutputData(Traj &traj)
{

   bool isReinterp=false;
   double outResT=_outRes;

   if(_outRes<_integRes)
   {
      isReinterp=true;
      _outRes=_integRes;
      _outSmoothFact*=std::max(outResT/_outRes,1.);
   }

   int nIntegPts=std::max((int)(_outSmoothFact*(1.1*traj.tTotalTraj/_outRes)),4);

   std::vector<double> sMVCout(nIntegPts),tMVCout;
   std::vector<double> dummy1,dummy2;
   int nPtsMVCout=0;

   Spline::splineSegs mySegs;
   // next: tInteg with variable-step
   double tLast=traj.tMVC[(int)traj.tMVC.size()-1];
   sMVCout=traj.sMVC; nPtsMVCout=(int)traj.sMVC.size();
   nPtsMVCout=(int)(_outSmoothFact*std::ceil(traj.tMVC[(int)traj.tMVC.size()-1]/_outRes+1.)); //+1
   nPtsMVCout=std::max(nPtsMVCout,4);
   tMVCout.resize(nPtsMVCout);

   // don't fix accel spike
   //std::iota(tMVCout.begin(), tMVCout.end(), 0);

   // to fix accel. spikes at the start and end of the traj.
   // this iscaused by output interpolation and not by integration:
   std::iota(tMVCout.begin(), tMVCout.end(), -1); //-1 for accel spike fix
   tMVCout[0]=0; //for accel spike fix
   tMVCout[1]=1.0/3.0; //for accel spike fix
   tMVCout[nPtsMVCout-1]=tMVCout[nPtsMVCout-2];
   tMVCout[nPtsMVCout-2]=tMVCout[nPtsMVCout-2]-1.0/3.0;

   tMVCout=traj.tMVC[(int)traj.tMVC.size()-1]/tMVCout[nPtsMVCout-1]*tMVCout;

   // interpolate traj. data at the new constant-time-resolution sites
   Spline::splineCoeffs sC;
   mySpline.findInterpSegs(traj.tMVC,tMVCout,mySegs);
   mySpline.getSplineCoeffs(traj.sMVC,sC,"natural");
   mySpline.interp1spline(sMVCout,dummy1,dummy2,
                          sC,mySegs,traj.sres/_outSmoothFact);

   mySpline.findInterpSegs(traj.sC,sMVCout,mySegs);
   traj.nPts=nPtsMVCout;

   traj.sres=_outRes;

   if(_pathType==JOINT || _pathType==BOTH)
   {
      for(unsigned int i=0; i<_nJoints; i++)
      {
         mySpline.interp1spline(traj.theta[i],
                                traj.thetaD[i],
                                traj.thetaD2[i],
                                traj.thetaC[i],
                                mySegs,traj.sres);
      }
      if(_pathType==JOINT && _robotType != GENJNT)
      {
         myRobot.call_fwdKin(traj.theta,traj.cart);
      }
   }
   if(_pathType==CART || _pathType==BOTH)
   {
      for(unsigned int i=0; i<_nCart; i++)
      {
         mySpline.interp1spline(traj.cart[i],
                                traj.cartD[i],
                                traj.cartD2[i],
                                traj.cartC[i],
                                mySegs,traj.sres);
      }
      if(_pathType==CART)
      {
         myRobot.call_invKin(traj.theta,traj.cart);
      }
   }

   if(_isTrqConOn)
   {
      mySegs.seg.resize(traj.nPts);
      std::iota(mySegs.seg.begin(), mySegs.seg.end(), -1); //was 0
      mySegs.tau.assign(traj.nPts,1);
      mySegs.seg[0]=0; mySegs.tau[0]=0;

      if(_isParallelMechOrig)
      {

         for(unsigned int i=0; i<_nJoints; i++)
         {
            mySpline.getSplineCoeffs(traj.theta[i],traj.thetaC[i],"natural");
            mySpline.interp1spline(traj.theta[i],traj.thetaD[i],traj.thetaD2[i],\
                                   traj.thetaC[i],mySegs,traj.sres/_outSmoothFact);
         }
         for(unsigned int i=0; i<_nCart; i++)
         {
            mySpline.getSplineCoeffs(traj.cart[i],traj.cartC[i],"natural");
            mySpline.interp1spline(traj.cart[i],traj.cartD[i],traj.cartD2[i],\
                                   traj.cartC[i],mySegs,traj.sres/_outSmoothFact);
         }


         traj.nPts=(int)traj.theta[0].size();

         myRobot.call_dynParallel(traj.a1,traj.a2,traj.a3,traj.a4,\
                                  traj.cart,traj.cartD,traj.cartD2);

         //bool isIllCond;
         std::vector<double> bStar(_nCart),xStar(_nCart);

         traj.trq.resize(_nJoints);
         for(unsigned int i=0; i<_nJoints; i++)
         {
            traj.trq[i].resize(traj.nPts);
         }
         std::vector<double> cartpt(_nCart),thetapt(_nJoints);
         for(unsigned int i=0; i<traj.nPts; i++)
         {
            for(unsigned int j=0; j<_nCart; j++)
            {
               bStar[j]=traj.a2[j][i]+traj.a3[j][i]+traj.a4[j][i];
            }
            for(unsigned int j=0; j<_nCart; j++)
            {
               cartpt[j]=traj.cart[j][i];
            }
            for(unsigned int j=0; j<_nJoints; j++)
            {
               thetapt[j]=traj.theta[j][i];
            }
            myRobot.call_setA(thetapt,cartpt,traj.Apt);

            solveLinSys(traj.Apt,bStar,xStar,_isSVD);
            for(unsigned int j=0; j<_nJoints; j++)
            {
               traj.trq[j][i]=xStar[j];
            }
         }
      }
      else
      {
         for(unsigned int i=0; i<_nJoints; i++)
         {
            mySpline.getSplineCoeffs(traj.theta[i],traj.thetaC[i],"clamped");
            mySpline.interp1spline(traj.theta[i],traj.thetaD[i],traj.thetaD2[i],
                                   traj.thetaC[i],mySegs,traj.sres/_outSmoothFact);
         }
         traj.nPts=(int)traj.theta[0].size();

         myRobot.call_dynSerial(traj.a1,traj.a2,traj.a3,traj.a4,
                                traj.theta,traj.thetaD,traj.thetaD2);
         traj.trq.resize(_nJoints);
         for(unsigned int i=0; i<_nJoints; i++)
         {
            traj.trq[i].resize(traj.nPts);
            for(unsigned int j=0; j<traj.nPts; j++)
            {
               traj.trq[i][j]=traj.a2[i][j]+traj.a3[i][j]+traj.a4[i][j];
            }
         }
      }
   }

   if(traj.cart[0].size() != traj.theta[0].size())
   {
      for(int i=0; i<3; i++)
      {
         traj.cart[i].resize(traj.nPts);
      }
   }


   if(_outSmoothFact>1.5) // conditionally smooth output data
   {
      int nIn=traj.nPts;
      int nOut=std::max((int)((nIn-1)/_outSmoothFact)+1,4);

      std::vector<double> inSites(nIn),outSites(nOut);
      std::iota(inSites.begin(), inSites.end(), 0);
      std::iota(outSites.begin(), outSites.end(), 0);
      outSites=inSites[nIn-1]/outSites[nOut-1]*outSites;
      mySpline.findInterpSegs(inSites,outSites,mySegs);

      int outSmoothFactInt=(int)_outSmoothFact;
      traj.nPts=nOut;
      for(unsigned int i=0; i<_nJoints; i++)
      {
         smooth(traj.theta[i],outSmoothFactInt);
         mySpline.interp1linear(traj.theta[i],mySegs);
      }

      if(_isTrqConOn)
      {
         for(unsigned int i=0; i<_nJoints; i++)
         {
            smooth(traj.trq[i],outSmoothFactInt);
            mySpline.interp1linear(traj.trq[i],mySegs);
         }
      }

      for(unsigned int i=0; i<_nCart; i++)
      {
         smooth(traj.cart[i],outSmoothFactInt);
         mySpline.interp1linear(traj.cart[i],mySegs);
      }
   }

   if(isReinterp)
   {
      int nPtsOut=std::max((int)(std::ceil(tLast/outResT)),4);
      if(nPtsOut==2)
      {
         tLast=outResT;
      }
      std::vector<double> s1(traj.nPts),s2(nPtsOut);

      std::iota(s1.begin(), s1.end(), 0);
      std::iota(s2.begin(), s2.end(), 0);
      s1=1./s1[traj.nPts-1]*s1;
      s2=1./s2[nPtsOut-1]*s2;

      mySpline.findInterpSegs(s1,s2,mySegs);

      for(unsigned int i=0; i<_nJoints; i++)
      {
         mySpline.getSplineCoeffs(traj.theta[i], traj.thetaC[i],"natural");

         mySpline.interp1spline(traj.theta[i], traj.thetaD[i], traj.thetaD2[i],\
                                traj.thetaC[i], mySegs,outResT);
      }
      if(!_isGenericRobot)
      {
         for(unsigned int i=0; i<_nCart; i++)
         {
            mySpline.getSplineCoeffs(traj.cart[i], traj.cartC[i],"natural");
            mySpline.interp1spline(traj.cart[i], traj.cartD[i],traj.cartD2[i],\
                                   traj.cartC[i], mySegs,outResT);
         }
      }
      if(_isTrqConOn)
      {

         for(unsigned int i=0; i<_nJoints; i++)
         {
            Spline::splineCoeffs dummyC;
            mySpline.getSplineCoeffs(traj.trq[i],dummyC,"natural");
            mySpline.interp1spline(traj.trq[i],dummy1,dummy2,\
                                   dummyC,mySegs,outResT);
         }
      }

      traj.nPts=nPtsOut;
      _outRes=outResT;
   }
   traj.sres=_outRes;
   traj.nPts=(int)traj.theta[0].size();
   if(_nCart==7)
   {
      q2aaVect(traj.cart);
      traj.cartC.resize(_nCart);
      traj.cartpt.resize(_nCart);
      traj.cartDpt.resize(_nCart);
      traj.cartD2pt.resize(_nCart);
   }
   return 0;
}

// *************** FILE IO functions ********************

/////////////////////////////////////////////////////////////////////////////
//!
//! \brief readConfigData
//!
//! read in config data file
//!
//////////////////////////////////////////////////////////////////////////////
int BA::readConfigData(const char* filename)
{
   std::string initialNumericLocale=std::setlocale(LC_NUMERIC, NULL);
   std::setlocale(LC_NUMERIC, "en_US.UTF-8");

   FILE *fid = fopen(filename, "r");
   if(fid == nullptr)
   {
      printf("\nUnable to open file %s\n", filename);
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }
   printf("\nConfiguration file: '%s'\n", filename);

   int readCountTotal = 0;

   for(int i=0; i<3; i++)
   {
      NextLine(fid);
   }

   _robotTypeStr = readChar(fid, readCountTotal);
   _isParallelMech = readBool(fid, readCountTotal);
   _isParallelMechOrig = _isParallelMech;
   _robotType = myRobot.call_set_robotType(_robotTypeStr);

   if(_robotTypeStr == "GENJNT")
   {
      _isGenericRobot=true;
   }
   else
   {
      _isGenericRobot=false;
   }
   if(_robotType==0)
   {
      fclose(fid);
      printf("\nreadInputData() error: robotType is %s",_robotTypeStr.c_str());
      printf("It should be 'KUKA', 'UR', 'RR', 'CSPR3DOF', or 'GENJNT'.\n");
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }
   _nJoints=readInt(fid, readCountTotal);
   _nCart   =readInt(fid, readCountTotal);

   char name[FILENAME_MAX];
   readCountTotal += fscanf(fid, "%s", name);
   _trajFileName = _InputFolder + name;

   NextLine(fid);
   _isBINfile=readBool(fid, readCountTotal);

   std::string pathTypeStr=readChar(fid, readCountTotal);

   _pathType=0;
   if(pathTypeStr=="JOINT") _pathType=JOINT;
   if(pathTypeStr=="CART")  _pathType=CART;
   if(pathTypeStr=="BOTH")  _pathType=BOTH;
   if(_pathType==0)
   {
      fclose(fid);
      printf("\nreadInputData() error: pathType is %s",pathTypeStr.c_str());
      printf("It should be 'JOINT', 'CART', or 'BOTH'.\n");
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }
   NextLine(fid); NextLine(fid);

   // CONSTRAINTS
   _areJointAnglesDegrees=readBool(fid,readCountTotal);
   _isJntVelConOn=readBool(fid,readCountTotal);
   _JntVelMax    =readDoubleVector(fid,readCountTotal,_nJoints);
   _isJntAccConOn=readBool(fid,readCountTotal);
   _JntAccMax    =readDoubleVector(fid,readCountTotal,_nJoints);
   _isTrqConOn   =readBool(fid,readCountTotal);
   _JntTrqMax    =readDoubleVector(fid,readCountTotal,_nJoints);
   _JntTrqMin    =readDoubleVector(fid,readCountTotal,_nJoints);

   for(unsigned int i=0; i<_nJoints; i++)
   {
      // if neg. torque constraints are NANs in the input file, symmetric
      // constraints will be used
      if(std::isnan(_JntTrqMin[i]))
      {
         _JntTrqMin[i]=-_JntTrqMax[i];
      }
   }
   _isCartVelConOn=readBool(fid,readCountTotal);
   _CartVelMax    =readDouble(fid,readCountTotal);
   _isCartAccConOn=readBool(fid,readCountTotal);
   _CartAccMax    =readDouble(fid,readCountTotal);

   NextLine(fid); NextLine(fid);

   // INTEGRATION PARAMETERS
   _integRes           =readDouble(fid,readCountTotal);
   _maxIntegTime       =readDouble(fid,readCountTotal);

   NextLine(fid); NextLine(fid);
   _inputDecimFact=readInt(fid,readCountTotal);
   _smoothWindow  =readInt(fid,readCountTotal);
   is_sdotOut      =readBool(fid,readCountTotal);
   _jntThresh        =readDouble(fid,readCountTotal);

   // INTERPOLATION PARAMETERS
   _cartThresh=readDouble(fid,readCountTotal);
   _quadraticRadThresh=_cartThresh*_cartThresh;

   _sWeights = readDoubleVector(fid,readCountTotal,3);
   _scaleType    =readInt(fid,readCountTotal);
   _thetaNormRes =readDouble(fid,readCountTotal);
   _thetaNormRes2=readDouble(fid,readCountTotal);
   _cartNormRes  =readDouble(fid,readCountTotal);
   _cartNormRes2 =readDouble(fid,readCountTotal);
   _outRes       =readDouble(fid,readCountTotal);
   _outSmoothFact=readDouble(fid,readCountTotal);
   _isSVD        =readBool(fid,readCountTotal);
   _isPar2Ser    =readBool(fid,readCountTotal);

   fclose(fid);

   double sWeightSum=_sWeights[0]+_sWeights[1]+_sWeights[2];
   if(sWeightSum<=0)
   {
      printf("Error in readInputData(): sum(sWeights) should be greater than 0.\n");
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }
   for(int i=0; i<3; i++)
   {
      _sWeights[i]/=sWeightSum;
   }

   int nReadItems=34+4*_nJoints;
   if(readCountTotal!=nReadItems)
   {
      printf("\nfscanf error while reading config.dat file: returned %d; should be %d.\n",
             readCountTotal,nReadItems);
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }

   std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL

   return 0;
}

////////////////////////////////////////////////////////////////////////////
//!
//! \brief loadConfigData
//!
//! Load the config data. This can be used rather than reading config data
//! from a file
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
int BA::loadConfigData(const Config &conf)
{
   _robotTypeStr = conf.robotTypeStr;
   _isParallelMech = conf.isParallelMech;
   _isParallelMechOrig = _isParallelMech;
   _robotType = myRobot.call_set_robotType(_robotTypeStr);

   if(_robotTypeStr == "GENJNT")
   {
      _isGenericRobot=true;
   }
   else{
      _isGenericRobot=false;
   }
   if(_robotType==0)
   {
      printf("\nreadInputData() error: robotType is %s",_robotTypeStr.c_str());
      printf("It should be 'KUKA', 'UR', 'RR', 'CSPR3DOF', or 'GENJNT'.\n");
      return -1;
   }
   _nJoints = conf.nJoints;
   _nCart   = conf.nCart;

   _trajFileName = conf.trajFileName;

   _isBINfile = conf.isBinFile;

   std::string pathTypeStr = conf.pathType;

   _pathType=0;
   if(pathTypeStr=="JOINT") _pathType=JOINT;
   if(pathTypeStr=="CART")  _pathType=CART;
   if(pathTypeStr=="BOTH")  _pathType=BOTH;
   if(_pathType==0)
   {
      printf("\nreadInputData() error: pathType is %s",pathTypeStr.c_str());
      printf("It should be 'JOINT', 'CART', or 'BOTH'.\n");
      return -1;
   }

   // CONSTRAINTS
   _isJntVelConOn = conf.isJntVelConon;
   _JntVelMax    = conf.jntVelLims;
   _isJntAccConOn= conf.isJntAccConOn;
   _JntAccMax    = conf.jntAccLims;
   _isTrqConOn   = conf.isTrqConOn;
   _JntTrqMax    = conf.jntTrqMax;
   _JntTrqMin    = conf.jntTrqMin;


   for(unsigned int i=0; i<_nJoints; i++)
   {
      // if neg. torque constraints are NANs in the input file, symmetric
      // constraints will be used
      if(std::isnan(_JntTrqMin[i])) _JntTrqMin[i]=-_JntTrqMax[i];
   }
   _isCartVelConOn=conf.isCartVelConOn;
   _CartVelMax    =conf.cartVelMax;
   _isCartAccConOn=conf.isCarAccConOn;
   _CartAccMax    =conf.cartAccMax;

   // INTERGRATION PARAMETERS
   _integRes           =conf.integRes;
   _maxIntegTime       =conf.maxIntegTime;

   _inputDecimFact=conf.inputDecimFact;
   _smoothWindow  =conf.smoothWindow;
   is_sdotOut      =conf.is_sdotOut;
   _jntThresh        =conf.jntThresh;

   // INTERPOLATION PARAMETERS
   _cartThresh=conf.cartThresh;
   _quadraticRadThresh=_cartThresh*_cartThresh;

   _sWeights = conf.sWeights;
   _scaleType    =conf.scaleType;
   _thetaNormRes =conf.thetaNormRes;
   _thetaNormRes2 =conf.thetaNormRes2;
   _cartNormRes  =conf.cartNormRes;
   _cartNormRes2 =conf.cartNormRes2;
   _outRes       =conf.outRes;
   _outSmoothFact=conf.outSmoothFact;
   _isSVD        =conf.isSVD;
   _isPar2Ser    =conf.isPar2Ser;

   double sWeightSum=_sWeights[0]+_sWeights[1]+_sWeights[2];
   if(sWeightSum<=0)
   {
      printf("Error in readInputData(): sum(sWeights) should be greater than 0.\n");
      return -1;
   }
   for(int i=0; i<3; i++)
   {
      _sWeights[i]/=sWeightSum;
   }

   return 0;
}

/////////////////////////////////////////////////////////////////////////////
//!
//! \brief loadTrajectoryData
//!
//! load the trajectory data
//!
//////////////////////////////////////////////////////////////////////////////
int BA::loadTrajectoryData(Traj &traj)
{
   int errorFlag = 0;

   traj.trajFileName = _trajFileName;

   traj.thetapt.resize(_nJoints);
   traj.thetaDpt.resize(_nJoints);
   traj.thetaD2pt.resize(_nJoints);
   traj.cartpt.resize(_nCart);
   traj.cartDpt.resize(_nCart);
   traj.cartD2pt.resize(_nCart);

   const char *trajFileNameC = _trajFileName.c_str();

   // 2. read in trajectory data
   if (doesFileExist(trajFileNameC) !=0 )
   {
      printf("Error: The file '%s' does not exist.\n", trajFileNameC);
      return -1;
   }

   if(_isBINfile)
   {
      errorFlag = trajReadBIN(traj, trajFileNameC);
   }
   else
   {
      errorFlag = trajReadCSV(traj, trajFileNameC);
   }

   if(errorFlag == -1)
   {
      return -1;
   }

   printInputData(traj);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief trajReadBIN
//!
//! read in a binary-format trajectory
//!
//! \param
//! \return
//!
//////////////////////////////////////////////////////////////////////////////
int BA::trajReadBIN(Traj &myTraj, const char* filename)
{

   int isThetaFull,isCartFull;
   size_t nReadCount=0;
   float tmpFloat1;

   FILE *fid=fopen(filename, "rb");
   if(fid == nullptr)
   {
      printf("\nError! Binary trajectory file '%s' doesn't exist!\n", filename);
      return -1;
   }
   nReadCount+=fread(&tmpFloat1,4,1,fid);
   myTraj.tresInput=(double)tmpFloat1;
   myTraj.sres=myTraj.tresInput;

   nReadCount+=fread(&myTraj.nPts,4,1,fid);
   std::vector<float> floatTemp(myTraj.nPts);

   //joint positions
   nReadCount+=fread(&isThetaFull,4,1,fid);
   if(isThetaFull==1)
   {

      myTraj.theta.resize(_nJoints, std::vector<double>(myTraj.nPts));
      for(unsigned int i=0; i<_nJoints; i++)
      {
         nReadCount+=fread(floatTemp.data(),4,myTraj.nPts,fid);
         std::copy(floatTemp.begin(),floatTemp.end(),myTraj.theta[i].begin());
      }
   }

   //Cart positions
   nReadCount+=fread(&isCartFull,4,1,fid);
   if(isCartFull==1)
   {
      myTraj.cart.resize(_nCart, std::vector<double>(myTraj.nPts));
      for(unsigned int i=0; i<_nCart; i++)
      {
         nReadCount+=fread(floatTemp.data(),4,myTraj.nPts,fid);
         std::copy(floatTemp.begin(),floatTemp.end(),myTraj.cart[i].begin());
      }
   }
   fclose(fid);

   int nReadCountNom=(isThetaFull*_nJoints+isCartFull*_nCart)*myTraj.nPts+4;
   if((int)nReadCount!=nReadCountNom)
   {
      // fread count doesn't match data set
      printf("\nfread error: %d items read, %d items should have been read.\n",
             (int)nReadCount,nReadCountNom);
      return -1;
   }
   return 0;
}


//////////////////////////////////////////////////////////////////////////////
//!
//! \brief trajReadCSV
//!
//! read in a CSV-format trajectory
//!
//////////////////////////////////////////////////////////////////////////////
int BA::trajReadCSV(Traj &traj, const char* filename)
{
   std::string initialNumericLocale=std::setlocale(LC_NUMERIC, NULL);
   std::setlocale(LC_NUMERIC, "en_US.UTF-8");

   FILE *fid = fopen(filename, "r");
   if(fid == nullptr)
   {
      printf("\nError! File %s doesn't exist", filename);
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }

   char tmpStr[20];
   size_t nFields;
   if(_isGenericRobot)
   {
       nFields = _nJoints;
   }
   else
   {
      nFields = _nJoints + _nCart + 1;
   }
   traj.trajFileHeader.resize(nFields);
   double tmpDouble;
   size_t fscanfReturnVal=0;

   NextLine(fid); //header

   // count the number of recorded points (lines that are not empty)

   traj.nPts = 0;
   while(1)
   {
      if((fscanfReturnVal=fscanf(fid, "%lf", &tmpDouble)) != 1)
      {
         break;
      }
      if(NextLine(fid) == EOF)
      {
         break;
      }
      traj.nPts++;
   }
   if(traj.nPts == 0)
   {
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return 0;
   }

   rewind(fid);

   bool isTimestamp=false;
   bool isJointData=false;
   bool isCartData =false;

   // read in the header fields
   fscanfReturnVal=0;
   for(size_t i=0; i < nFields; i++)
   {
      fscanfReturnVal+=fscanf(fid," %99[^, \t\n],",tmpStr);
      traj.trajFileHeader[i]=tmpStr;
      if(traj.trajFileHeader[i]=="timestamp")
      {
         isTimestamp=true;
      }
      if(traj.trajFileHeader[i]=="j1")
      {
         isJointData=true;
      }
      if(traj.trajFileHeader[i]=="x")
      {
         isCartData =true;
      }
   }

   // read in the joint and Cartesian trajectory data
   traj.timestamp.resize(traj.nPts);
   if(isJointData)
   {
      traj.theta.resize(_nJoints, std::vector<double>(traj.nPts));
      for(auto& j: traj.theta)
      {
         j.resize(traj.nPts, 0);
      }
   }
   if(isCartData)
   {
      traj.cart.resize(_nCart, std::vector<double>(traj.nPts));
      for(auto& p: traj.cart)
      {
         p.resize(traj.nPts, 0);
      }
   }

   for(size_t i=0; i<traj.nPts; i++)
   {
      if(isTimestamp)
      {
         fscanfReturnVal+=fscanf(fid, "%lf,", &traj.timestamp[i]);
      }
      if(isJointData)
      {
         for(size_t j=0; j<_nJoints;j++)
         {
            fscanfReturnVal+=fscanf(fid,"%lf,",&traj.theta[j][i]);
         }
      }
      if(isCartData)
      {
         for(size_t j=0; j<_nCart; j++)
         {
            fscanfReturnVal+=fscanf(fid,"%lf,",&traj.cart[j][i]);
         }
      }
   }
   fclose(fid);

   if(!isTimestamp)
   {
       std::iota(traj.timestamp.begin(), traj.timestamp.end(), 0);
       traj.timestamp = 0.2*traj.timestamp;
   }

   traj.tresInput = traj.timestamp.back()/(traj.nPts-1);
   traj.sres = traj.tresInput;

   if(nFields*(traj.nPts+1) != fscanfReturnVal)
   {
      printf("trajReadCSV: The number of items read from %s was %d. It should have been %d.\n",
             filename, (int)fscanfReturnVal, (int)nFields*(traj.nPts+1));
      printf("Most likely the run environment is not EN_US and fscanf is expecting commas for the decimal.\n");
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }

   std::setlocale(LC_NUMERIC, initialNumericLocale.c_str()); //LC_ALL

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief printInputData
//!
//! print out selected configuration and trajectory data to the console
//!
//////////////////////////////////////////////////////////////////////////////
int BA::printInputData(const Traj &myTraj)
{

   printf("\n");
   printf("Robot: %s \n", _robotTypeStr.c_str());
   printf("Number of robot joints: %u \n", _nJoints);
   printf("Input  traj. file : %s\n", myTraj.trajFileName.c_str());
   printf("Input resolution  :  %.4f s\n", myTraj.tresInput);
   printf("Number of traj pts: %d\n", myTraj.nPts);

   printf("Joint velocity limits : ");
   for(unsigned int i=0; i<_nJoints; i++)
   {
      printf("%.1f ",_JntVelMax[i]);
   }
   printf("\n");

   printf("Joint accel.   limits : ");
   for(unsigned int i=0; i<_nJoints; i++)
   {
      printf("%.1f ",_JntAccMax[i]);
   }
   printf("\n");

   printf("Cartesian speed  limit: %.4f\n"  ,_CartVelMax);
   printf("Integration resolution: %.4f s\n",_integRes);
   printf("Output      resolution: %.4f s\n",_outRes);
   printf("Max. integration time : %.0f s\n",_maxIntegTime);
   printf("\n");

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief writeOutputData
//!
//! write output data files
//!
//////////////////////////////////////////////////////////////////////////////
int BA::writeOutputData(Traj &myTraj)
{
   std::string filename = _OutputFolder + "traj_out.dat";
   trajWriteBIN(myTraj,filename.c_str());
   if(!_isBINfile)
   {
     filename = _OutputFolder + "traj_out.csv";
      trajWriteCSV(myTraj, filename.c_str());
   }
   if(is_sdotOut && !_isInterpOnly)
   {
     filename = _OutputFolder + "s-sdot.dat";
      sdotWrite(myTraj, filename.c_str());
   }

   printf("\nOutput trajectory is %.3f sec.\n",(myTraj.nPts-1)*myTraj.sres);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief optimize
//!
//! call the functions for interpolating input data, integrating accel. curves
//! and interpolating output data
//!
//////////////////////////////////////////////////////////////////////////////
int BA::optimize(Traj &traj)
{
   int errorFlag;
   setErrorOptimization(NO_ERROR);

   errorFlag = interpInputData(traj);
   if(errorFlag == -1)
   {
      return -1;
   }
   if(traj.nPts < 4)
   {
      return -1;
   }

   setIntegDir(-1);
   setIsLastSweep(false);

   errorFlag = sweep(traj);
   if(errorFlag == -1)
   {
      return -1;
   }

   setIntegDir(1);
   setIsLastSweep(true);
   errorFlag=sweep(traj);
   if(errorFlag == -1)
   {
      return -1;
   }

   interpOutputData(traj);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief trajWriteBIN
//!
//! Output trajectory data to a binary file
//!
//////////////////////////////////////////////////////////////////////////////
int BA::trajWriteBIN(Traj &myTraj,const char *fname)
{
   FILE *fid=fopen(fname,"wb");
   if(fid == nullptr)
   {
      printf("\nUnable to open file %s", fname);
      return -1;
   }

   size_t nPts;
   float floatTemp1;

   int is_trqFull=0;
   int is_cartFull=0;
   int is_thetaFull=1;

   if(myTraj.theta.empty())
   {
      printf("trajWrite(): myTraj is empty; no file was written.\n");
      return -1;
   }

   nPts=myTraj.theta[0].size();
   //float floatTemp[nPts];

   if(myTraj.cart.size()==_nCart)
   {
      if(myTraj.cart[0].size()==nPts) is_cartFull=1;
   }
   if(_isTrqConOn && !myTraj.trq.empty())
   {
      if(!myTraj.trq[0].empty()) is_trqFull=1;
   }

   // path indexing
   floatTemp1=(float)myTraj.sres;
   fwrite(&floatTemp1, 4, 1, fid);
   fwrite(&myTraj.nPts, 4, 1, fid);

   fwrite(&is_thetaFull, 4, 1, fid);
   // path data

   std::vector<float> floatTemp(nPts);
   for(size_t i=0; i<_nJoints; i++)
   {
      std::copy(myTraj.theta[i].begin(), myTraj.theta[i].end(),floatTemp.begin());
      fwrite(floatTemp.data(), 4, nPts, fid);
   }

   fwrite(&is_cartFull,4,1,fid);
   if(is_cartFull==1)
   {
      for(size_t i=0; i<_nCart; i++)
      {
         std::copy(myTraj.cart[i].begin(), myTraj.cart[i].end(),floatTemp.begin());
         fwrite(floatTemp.data(),4,nPts,fid);
      }
   }
   fwrite(&is_trqFull,4,1,fid);
   if(is_trqFull)
   {
      for(size_t i=0; i<_nJoints; i++)
      {
         std::copy(myTraj.trq[i].begin(), myTraj.trq[i].end(),floatTemp.begin());
         fwrite(floatTemp.data(),4,nPts,fid);
      }
   }
   fclose(fid);
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief trajWriteCSV
//!
//! Output a trajectory in CSV format
//!
//////////////////////////////////////////////////////////////////////////////
int BA::trajWriteCSV(Traj &myTraj, const char *fname)
{
   std::string initialNumericLocale=std::setlocale(LC_NUMERIC, NULL);
   std::setlocale(LC_NUMERIC, "en_US.UTF-8");

   FILE *fid=fopen(fname,"w");
   if(fid == nullptr)
   {
      printf("\nUnable to open file %s", fname);
      std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
      return -1;
   }

   for(unsigned int i=0; i<myTraj.trajFileHeader.size()-1; i++)
   {
      fprintf(fid,"%s, ", myTraj.trajFileHeader[i].c_str());
   }
   fprintf(fid, "%s\n", myTraj.trajFileHeader[myTraj.trajFileHeader.size()-1].c_str());

   if(myTraj.nPts != myTraj.timestamp.size())
   {
      _isInterpolated = true;
   }

   int is_cartFull = 0;
   if (myTraj.cart.size() == _nCart)
   {
      if (myTraj.cart[0].size() == myTraj.nPts) is_cartFull = 1;
   }

   for(unsigned int i=0; i<myTraj.nPts; i++)
   {
      if(_isInterpolated)
      {
         fprintf(fid, "%8.3f", i*myTraj.sres);
      }
      else
      {
         fprintf(fid, "%8.3f", myTraj.timestamp[i]);
      }

      for(unsigned int j=0; j<_nJoints; j++)
      {
         fprintf(fid, ", %11.6f", myTraj.theta[j][i]);
      }
      if(is_cartFull == 1)
      {
         for (unsigned int j = 0; j < _nCart; j++)
         {
            fprintf(fid, ", %9.6f", myTraj.cart[j][i]);
         }
      }
      fprintf(fid, "\n");
   }
   fclose(fid);
   std::setlocale(LC_NUMERIC,initialNumericLocale.c_str()); //LC_ALL
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief sdotWrite
//!
//! Output maximum-velocity curve (MVC) data to a binary file
//!
//////////////////////////////////////////////////////////////////////////////
int BA::sdotWrite(Traj &myTraj,const char *fname)
{
   FILE *fid=fopen(fname,"wb");
   if(fid == nullptr)
   {
      printf("\nUnable to open file %s", fname);
      return -1;
   }

   for(int i=0; i<2; i++)
   {
      int nPts=(int)myTraj.myMVChist.s[i].size();

      if(nPts > 0)
      {
         std::vector<float> floatTemp(nPts);
         fwrite(&myTraj.sres, 8, 1, fid);
         fwrite(&nPts, 4, 1, fid);

         std::copy(myTraj.myMVChist.s[i].begin(), myTraj.myMVChist.s[i].end(),floatTemp.begin());
         fwrite(floatTemp.data(), 4, nPts, fid);
         std::copy(myTraj.myMVChist.sdot[i].begin(), myTraj.myMVChist.sdot[i].end(),floatTemp.begin());
         fwrite(floatTemp.data(), 4, nPts, fid);
      }
      else
      {
         printf("sdotWrite(): %s was not written because sdot is empty.\n",fname);
         return -1;
      }
   }
   fclose(fid);

   return 0;
}

//////////////////////////////////////////////////////////////////////////////
//!
//! \brief interpTrajLinear
//!
//! Linear interpolation of a trajectory
//!
//////////////////////////////////////////////////////////////////////////////
int BA::interpTrajLinear(Traj &traj,const int nPtsNew)
{
   int nPtsOld=traj.nPts;
   std::vector<double> ptsOld(nPtsOld);
   std::iota(ptsOld.begin(), ptsOld.end(), 0);
   ptsOld=1.0/(nPtsOld-1)*ptsOld;

   std::vector<double> ptsNew(nPtsNew);
   std::iota(ptsNew.begin(), ptsNew.end(), 0);
   ptsNew=1.0/(nPtsNew-1)*ptsNew;

   Spline::splineSegs mySegs;
   mySpline.findInterpSegs(ptsOld,ptsNew,mySegs);

   for(unsigned int i=0;i<_nJoints;i++)
   {
      mySpline.interp1linear(traj.theta[i],mySegs);
   }
   for(unsigned int i=0;i<_nCart;i++)
   {
      mySpline.interp1linear(traj.cart[i],mySegs);
   }
   traj.sres=traj.sres*(nPtsOld-1)/(nPtsNew-1);
   traj.nPts=nPtsNew;

   return 0;
}

}//namespace BATOTP
