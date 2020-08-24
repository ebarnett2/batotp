////////////////////////////////////////////////////////////////////////////////
//! \file    main.cpp
//! \date    Sep 2017
//! \author  Eric Barnett
//!
//! \brief main source file for generating the ba.exe executable
//!
//! Copyright (C) 2017 - All Rights Reserved
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "ba.h"
#include "util.h"

using namespace BATOTP;

int main(void)
{

   int errorFlag;
   BA myBA;
   Traj myTraj;
   std::vector<Time>t(6);

   t[0]=getTime();

   errorFlag=myBA.readConfigData("./input/config.dat");
   if(errorFlag==-1)
   {
      return -1;
   }

   errorFlag=myBA.loadTrajectoryData(myTraj, "./input/trajUR.csv");
   if(errorFlag==-1)
   {
      return -1;
   }

   t[1]=getTime();
   printf("-----Interpolation of input data-----------\n");
   errorFlag=myBA.interpInputData(myTraj);
   if(errorFlag==-1)
   {
      return -1;
   }

   t[2]=getTime();
   printf("\n\n--Constant-step accel. constraint integ.--");
   myBA.setIntegDir(-1);
   myBA.setIsLastSweep(false);
   errorFlag=myBA.sweep(myTraj);
   if(errorFlag==-1)
   {
      return -1;
   }
   myBA.setIntegDir(1);
   myBA.setIsLastSweep(true);
   errorFlag=myBA.sweep(myTraj);
   if(errorFlag==-1)
   {
      return -1;
   }
   t[3]=getTime();
   printf("---------------------------------------\n");
   myBA.interpOutputData(myTraj);

   t[4]=getTime();
   myBA.writeOutputData(myTraj);

   t[5]=getTime();

   printf("\nComputational times (sec):\n");
   printf("Input data interp.      : %f\n",diffTime(t[2],t[1]));
   printf("Accel. constraint integ.: %f\n",diffTime(t[3],t[2]));
   printf("Reading input data      : %f\n",diffTime(t[1],t[0]));
   printf("Writing Output data     : %f\n",diffTime(t[5],t[4]));
   printf("Total,     with file IO : %f\n",diffTime(t[5],t[0]));
   printf("Total,  without file IO : %f\n",diffTime(t[4],t[1]));
   printf("\n");

   FILE *fid;
   float integTime;
   fid=fopen("./output/compTimes.dat","wb");

   integTime=(float)diffTime(t[3],t[2]);
   fwrite(&integTime,4,1,fid);

   integTime=(float)diffTime(t[4],t[1]);
   fwrite(&integTime,4,1,fid);

   integTime=(float)diffTime(t[5],t[0]);
   fwrite(&integTime,4,1,fid);

   fclose(fid);

   return 0;
}
