/**
 * This file is part of DSO, written by Jakob Engel.
 * It has been modified by Lukas von Stumberg for the inclusion in DM-VIO (http://vision.in.tum.de/dm-vio).
 *
 * Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <string.h>
#include <string>
#include <cmath>
#include "util/NumType.h"

namespace dso
{
#define SOLVER_SVD (int)1
#define SOLVER_ORTHOGONALIZE_SYSTEM (int)2
#define SOLVER_ORTHOGONALIZE_POINTMARG (int)4
#define SOLVER_ORTHOGONALIZE_FULL (int)8
#define SOLVER_SVD_CUT7 (int)16
#define SOLVER_REMOVE_POSEPRIOR (int)32
#define SOLVER_USE_GN (int)64
#define SOLVER_FIX_LAMBDA (int)128
#define SOLVER_ORTHOGONALIZE_X (int)256
#define SOLVER_MOMENTUM (int)512
#define SOLVER_STEPMOMENTUM (int)1024
#define SOLVER_ORTHOGONALIZE_X_LATER (int)2048

// ============== PARAMETERS TO BE DECIDED ON COMPILE TIME =================
#define PYR_LEVELS 6

  struct GlobalCalib
  {
    GlobalCalib() = default;
    GlobalCalib(int w, int h, const Eigen::Matrix3f &K, int &pyrLevelsUsed);

    int wG[PYR_LEVELS], hG[PYR_LEVELS];
    float fxG[PYR_LEVELS], fyG[PYR_LEVELS],
        cxG[PYR_LEVELS], cyG[PYR_LEVELS];

    float fxiG[PYR_LEVELS], fyiG[PYR_LEVELS],
        cxiG[PYR_LEVELS], cyiG[PYR_LEVELS];

    Eigen::Matrix3f KG[PYR_LEVELS], KiG[PYR_LEVELS];

    float wM3G, hM3G;
  };

  struct Settings
  {
    int pyrLevelsUsed = PYR_LEVELS;

    bool useIMU = true;              // Use IMU data (false will disable all IMU integration).
    bool useGTSAMIntegration = true; // Use the GTSAM integration for integrating addtional factors to the BA. Needed when useIMU==true).

    // If non-zero we set a prior to the x or y direction of the translation during the coarse visual initializer (useful for car datasets).
    double weightZeroPriorDSOInitY = 0.0;
    double weightZeroPriorDSOInitX = 0.0;
    double forceNoKFTranslationThresh = 0.0; // Force to create no KF if translation (in metric) is smaller than this.

    double maxTimeBetweenKeyframes = 0;

    // If negative, the respective positive value will be used only if in non-RT mode.
    // The idea of this parameter is that in non-RT mode the systems otherwise can make successive frames keyframes, which only rarely happens in RT mode.
    // Default is -0.5 with means that the parameter is 0.5 in non-RT mode and inactive in RT mode.
    // Fractional values are also possible.
    double minFramesBetweenKeyframes = -0.5;

    // minimum idepth for keeping points in the optimization window.
    float minIdepth = 0.02f;

    /* Parameters controlling when KF's are taken */
    float keyframesPerSecond = 0; // if !=0, takes a fixed number of KF per second.
    bool realTimeMaxKF = false;   // if true, takes as many KF's as possible (will break the system if the camera stays stationary)
    float maxShiftWeightT = 0.04f * (640 + 480);
    float maxShiftWeightR = 0.0f * (640 + 480);
    float maxShiftWeightRT = 0.02f * (640 + 480);
    float kfGlobalWeight = 1; // general weight on threshold, the larger the more KF's are taken (e.g., 2 = double the amount of KF's).
    float maxAffineWeight = 2;

    /* initial hessian values to fix unobservable dimensions / priors on affine lighting parameters.
     */
    float idepthFixPrior = 50 * 50;          // * 1000;
    float idepthFixPriorMargFac = 600 * 600; // 30000*30000;
    float initialRotPrior = 1e11;            // 5e7;// 1e11;
    float initialTransPrior = 1e10;          // 1e10;
    float initialAffBPrior = 1e14;
    float initialAffAPrior = 1e14;
    float initialCalibHessian = 5e9;

    /* some modes for solving the resulting linear system (e.g. orthogonalize wrt. unobservable dimensions) */
    // int solverMode = SOLVER_FIX_LAMBDA | SOLVER_ORTHOGONALIZE_X_LATER;
    int solverMode = SOLVER_ORTHOGONALIZE_X_LATER;
    double solverModeDelta = 0.00001;
    bool forceAceptStep = false;

    /* some thresholds on when to activate / marginalize points */
    float minIdepthH_act = 100;
    float minIdepthH_marg = 50;

    float desiredImmatureDensity = 1500; // immature points per frame
    float desiredPointDensity = 2000;    // aimed total points in the active window.
    float minPointsRemaining = 0.05;     // marg a frame if less than X% points remain.
    float maxLogAffFacInWindow = 0.7;    // marg a frame if factor between intensities to current frame is larger than 1/X or X.

    int minFrames = 5; // min frames in window.
    int maxFrames = 7; // max frames in window.
    int minFrameAge = 1;
    int maxOptIterations = 6;    // max GN iterations.
    int minOptIterations = 1;    // min GN iterations.
    float thOptIterations = 1.2; // factor on break threshold for GN iteration (larger = break earlier)

    /* Outlier Threshold on photometric energy */
    float outlierTH = 12 * 12;             // higher -> less strict
    float outlierTHSumComponent = 50 * 50; // higher -> less strong gradient-based reweighting .

    int pattern = 8;                 // point pattern used. DISABLED.
    float margWeightFac = 0.5 * 0.5; // factor on hessian when marginalizing, to account for inaccurate linearization points.

    /* when to re-track a frame */
    float reTrackThreshold = 1.5; // (larger = re-track more often)

    /* require some minimum number of residuals for a point to become valid */
    int minGoodActiveResForMarg = 3;
    int minGoodResForMarg = 4;

    // 0 = nothing.
    // 1 = apply inv. response.
    // 2 = apply inv. response & remove V.
    int photometricCalibration = 2;
    bool useExposure = true;
    float affineOptModeA = 1e12; //-1: fix. >=0: optimize (with prior, if > 0).
    float affineOptModeB = 1e8;  //-1: fix. >=0: optimize (with prior, if > 0).
    float affineOptModeA_huberTH = 10000;
    float affineOptModeB_huberTH = 10000;
    int gammaWeightsPixelSelect = 1; // 1 = use original intensity for pixel selection; 0 = use gamma-corrected intensity.

    float huberTH = 9; // Huber Threshold

    // parameters controlling adaptive energy threshold computation.
    float frameEnergyTHConstWeight = 0.5;
    float frameEnergyTHN = 0.7f;
    float frameEnergyTHFacMedian = 1.5;
    float overallEnergyTHWeight = 1;
    float coarseCutoffTH = 20;

    // parameters controlling pixel selection
    float minGradHistCut = 0.5;
    float minGradHistAdd = 7;
    float gradDownweightPerLevel = 0.75;
    bool selectDirectionDistribution = true;

    /* settings controling initial immature point tracking */
    float maxPixSearch = 0.027; // max length of the ep. line segment searched during immature point tracking. relative to image resolution.
    float minTraceQuality = 3;
    int minTraceTestRadius = 2;
    int GNItsOnPointActivation = 3;
    float trace_stepsize = 1.0;           // stepsize for initial discrete search.
    int trace_GNIterations = 3;           // max # GN iterations
    float trace_GNThreshold = 0.1;        // GN stop after this stepsize.
    float trace_extraSlackOnTH = 1.2;     // for energy-based outlier check, be slightly more relaxed by this factor.
    float trace_slackInterval = 1.5;      // if pixel-interval is smaller than this, leave it be.
    float trace_minImprovementFactor = 2; // if pixel-interval is smaller than this, leave it be.

    // for benchmarking different undistortion settings
    float benchmark_fxfyfac = 0;
    int benchmark_width = 0;
    int benchmark_height = 0;
    float benchmark_varNoise = 0;
    float benchmark_varBlurNoise = 0;
    float benchmark_initializerSlackFactor = 1;
    int benchmark_noiseGridsize = 3;

    float freeDebugParam1 = 1;
    float freeDebugParam2 = 1;
    float freeDebugParam3 = 1;
    float freeDebugParam4 = 1;
    float freeDebugParam5 = 1;

    bool debugSaveImages = false;
    bool multiThreading = true;
    bool disableAllDisplay = false;
    bool logStuff = true;

    bool goStepByStep = false;

    bool render_displayCoarseTrackingFull = false;
    bool render_renderWindowFrames = true;
    bool render_plotTrackingFull = false;
    bool render_display3D = true;
    bool render_displayResidual = true;
    bool render_displayVideo = true;
    bool render_displayDepth = true;

    bool fullResetRequested = false;

    bool debugout_runquiet = false;

    int multiCameraIndex = 0;

    int sparsityFactor = 5; // not actually a setting, only some legacy stuff for coarse initializer.

    float wTarget = 0;
    float hTarget = 0;

    GlobalCalib calibG;

    void handleKey(char k)
    {
      char kkk = k;
      switch (kkk)
      {
      case 'd':
      case 'D':
        freeDebugParam5 = ((int)(freeDebugParam5 + 1)) % 10;
        printf("new freeDebugParam5: %f!\n", freeDebugParam5);
        break;
      case 's':
      case 'S':
        freeDebugParam5 = ((int)(freeDebugParam5 - 1 + 10)) % 10;
        printf("new freeDebugParam5: %f!\n", freeDebugParam5);
        break;
      }
    }
  };

  extern int staticPattern[13][40][2];
  extern int staticPatternNum[13];
  extern int staticPatternPadding[13];

  const int pattern = 8;
  const int patternNum = staticPatternNum[pattern];
  const auto patternP = staticPattern[pattern];
  const int patternPadding = staticPatternPadding[pattern];

}
