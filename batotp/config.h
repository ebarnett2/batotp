////////////////////////////////////////////////////////////////////////////
//! \file    config.h
//! \date    February 2018
//! \author  Eric Barnett
//!          Marc-Antoine Lacasse
//!
//! Copyright (C) 2018 Eric Barnett         <e.barnett@robotiq.com>
//! Copyright (C) 2018 Marc-Antoine Lacasse <malacasse@robotiq.com>
//! Copyright (C) 2018 Clément Gosselin     <gosselin@gmc.ulaval.ca>
//!
//! \brief   Declares global constants
//!
//! This file is part of BA, a bisection algorithm for time-optimal trajectory
//! planning
//!
//! This Source Code Form is subject to the terms of the Berkeley Software
//! Distribution (BSD) 3-clause license . If a copy of the BSD license was not
//! distributed with this file, you can obtain one at
//! https://opensource.org/licenses/BSD-3-Clause.
////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_H
#define CONFIG_H

#define _CONFIG_FILE "./input/config.dat"

static const double PI=3.14159265358979323846;
static const double _DEG2RAD=PI/180.0;
static const double _RAD2DEG=180.0/PI;
static const double _g=9.81;

static const int _MAXCHAR=100;

#endif //CONFIG_H
