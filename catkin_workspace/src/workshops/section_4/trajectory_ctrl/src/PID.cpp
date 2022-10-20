// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

/**
 * @file PID.cpp
 * @author Guido Küppers
 * @brief  Discrete PID-Controller Class.
 */
#include "PID.h"

// Constructor
PID::PID(double Kp, double Ki, double Kd)
{
    this->SetParameters(Kp, Ki, Kd);
}

// Setter function for paramters
void PID::SetParameters(double Kp, double Ki, double Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    this->Reset();
}

// Reset function
void PID::Reset()
{
    i_val_ = 0.0;
    pre_e_ = nan("");
}

// Actual calculation for a discrete time-step
double PID::Calc(double e, double dt)
{
    i_val_ += e * dt;
    double d_val = 0.0;
    if (std::isnan(pre_e_))
    {
        pre_e_ = e;
    }
    if (dt != 0.0)
        d_val = (e - pre_e_) / dt;

    pre_e_ = e;

    // START TASK 3 CODE HERE
    // use helping comments from Wiki
    return 0.0;
    // END TASK 3 CODE HERE
}
