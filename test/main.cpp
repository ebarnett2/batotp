////////////////////////////////////////////////////////////////////////////
//! \file    main.cpp
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief  main source file for generating the batotp executable
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

using namespace BATOTP;

int main(int argc, char *argv[])
{
   int errorFlag;
   BA myBA;
   Traj myTraj;
   std::vector<Time>t(6);

   std::string configFileIn;

   if(argc>1) // config file from the terminal argument
   {
      configFileIn=argv[1];
      myBA.setHomeFolder("./");
      myBA.setInputFolder("./");
      myBA.setOutputFolder("./");
   }
   else // config file located in the ../input folder
   {
      configFileIn = "config.dat";
      mkDirIfNec(myBA.getOutputFolder().c_str());
   }

   std::string filename = myBA.getInputFolder() + configFileIn;
   myBA.setIsAutoIntegRes(false);

   t[0]=getTime();

   errorFlag=myBA.readConfigData(filename.c_str());
   if(errorFlag==-1) return -1;

   errorFlag=myBA.loadTrajectoryData(myTraj);
   if(errorFlag==-1) return -1;

   t[1]=getTime();
   printf("-----Interpolation of input data-----------\n");
   errorFlag=myBA.interpInputData(myTraj);
   if(errorFlag==-1) return -1;

   t[2]=getTime();
   printf("\n--Constant-step accel. constraint integ.--\n");
   myBA.setIntegDir(-1);
   myBA.setIsLastSweep(false);
   errorFlag=myBA.sweep(myTraj);
   if(errorFlag==-1) return -1;

   myBA.setIntegDir(1);
   myBA.setIsLastSweep(true);
   errorFlag=myBA.sweep(myTraj);
   if(errorFlag==-1) return -1;
   
   t[3]=getTime();
   printf("---------------------------------------\n");
   myBA.interpOutputData(myTraj);

   t[4]=getTime();
   myBA.writeOutputData(myTraj);
   t[5]=getTime();

   // PRINT COMPUTATION TIMES TO CONSOLE AND TO FILE

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
   filename = myBA.getOutputFolder() + "compTimes.dat";
   fid=fopen(filename.c_str(),"wb");

   integTime=(float)diffTime(t[3],t[2]);
   fwrite(&integTime,4,1,fid);

   integTime=(float)diffTime(t[4],t[1]);
   fwrite(&integTime,4,1,fid);

   integTime=(float)diffTime(t[5],t[0]);
   fwrite(&integTime,4,1,fid);

   fclose(fid);

   return 0;
}
