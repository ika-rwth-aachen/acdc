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

/** @addtogroup IKA_DEFINITION */
/**@{*/
#pragma once

/**
 * @brief This namespace will define the minimal sizes of all sensor objects.
 */
namespace defBoxSize {

/**
 * @brief Minimal Delphi ESR length.
 */
static const float ESR_X = 1.0f;
/**
 * @brief Minimal Delphi ESR width.
 */
static const float ESR_Y = 1.0f;

/**
 * @brief Minimal Mobileye EPM length.
 */
static const float EPM_X = 0.5f;
/**
 * @brief Minimal Mobileye EPM width.
 */
static const float EPM_Y = 0.5f;

/**
 * @brief Minimal Ibeo Lux length.
 */
static const float LUX_X = 0.5f;
/**
 * @brief Minimal Ibeo Lux width.
 */
static const float LUX_Y = 0.5f;

/**
 * @brief Minimal Delphi SRR length.
 */
static const float SRR_X = 0.1f;
/**
 * @brief Minimal Delphi SRR width.
 */
static const float SRR_Y = 0.1f;

/**
 * @brief Minimal Ibeo Scala length.
 */
static const float SCALA_X = 0.01f;
/**
 * @brief Minimal Ibeo Scala width.
 */
static const float SCALA_Y = 0.01f;

/**
 * @brief Fix Height of all objects.
 */
static const float UNI_Z = 1.5f;

}

/**
 * @brief Default width of a lane if no other information is available.
 */
static const double DEFAULT_LANE_WIDTH = 3.0;
/**
 * @brief Is this needed anymore?
 * @todo What does this quantity has done, does this feature got lost?
 */
static const bool RT_IN_ALL_OBJECTLIST = false;
/**@}*/
