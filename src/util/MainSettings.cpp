/**
 * This file is part of DM-VIO.
 *
 * Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
 * for more information see <http://vision.in.tum.de/dm-vio>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * The methods parseArgument and settingsDefault are based on the file
 * main_dso_pangolin.cpp of the project DSO written by Jakob Engel, but have been
 * modified for the inclusion in DM-VIO. The original versions have the copyright
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 *
 * DM-VIO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DM-VIO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
 */

#include "MainSettings.h"
#include "dso/util/settings.h"

using namespace dmvio;
using namespace dso;

void MainSettings::parseArguments(int argc, char **argv, SettingsUtil &settingsUtil)
{
  for (int i = 1; i < argc; i++)
    parseArgument(argv[i], settingsUtil);
}

void MainSettings::parseArgument(char *arg, SettingsUtil &settingsUtil)
{
  int option;
  float foption;
  char buf[1000];

  // --------------------------------------------------
  // These are mostly the original DSO commandline arguments which also work for DM-VIO.
  // The DM-VIO specific settings can also be set with commandline arguments (and also with the yaml settings file)
  // and have been registered in the main function. See IMUSettings and its members for details.
  // --------------------------------------------------
  if (1 == sscanf(arg, "quiet=%d", &option))
  {
    if (option == 1)
    {
      settings->debugout_runquiet = true;
      printf("QUIET MODE, I'll shut up!\n");
    }
    return;
  }

  if (1 == sscanf(arg, "preset=%d", &option))
  {
    settingsDefault(option);
    return;
  }

  if (1 == sscanf(arg, "nolog=%d", &option))
  {
    if (option == 1)
    {
      settings->logStuff = false;
      printf("DISABLE LOGGING!\n");
    }
    return;
  }
  if (1 == sscanf(arg, "nogui=%d", &option))
  {
    if (option == 1)
    {
      settings->disableAllDisplay = true;
      printf("NO GUI!\n");
    }
    return;
  }
  if (1 == sscanf(arg, "nomt=%d", &option))
  {
    if (option == 1)
    {
      settings->multiThreading = false;
      printf("NO MultiThreading!\n");
    }
    return;
  }

  if (1 == sscanf(arg, "useimu=%d", &option))
  {
    if (option == 0)
    {
      printf("Disabling IMU integration!\n");
      settings->useIMU = false;
    }
    else if (option == 1)
    {
      printf("Enabling IMU integration!\n");
      settings->useIMU = true;
    }
    return;
  }

  if (1 == sscanf(arg, "save=%d", &option))
  {
    if (option == 1)
    {
      settings->debugSaveImages = true;
      if (42 == system("rm -rf images_out"))
        printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      if (42 == system("mkdir images_out"))
        printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      if (42 == system("rm -rf images_out"))
        printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      if (42 == system("mkdir images_out"))
        printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      printf("SAVE IMAGES!\n");
    }
    return;
  }

  if (1 == sscanf(arg, "mode=%d", &option))
  {

    mode = option;
    if (option == 0)
    {
      printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
    }
    if (option == 1)
    {
      printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
      settings->photometricCalibration = 0;
      settings->affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
      settings->affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    }
    if (option == 2)
    {
      printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
      settings->photometricCalibration = 0;
      settings->affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
      settings->affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
      settings->minGradHistAdd = 3;
    }
    if (option == 3)
    {
      // This mode is useful because mode 0 assumes that exposure is available (as it adds a strong prior to
      // the affine brightness change between images), and mode 1 does not use vignette at all.
      // This mode uses vignette (and response), but still fully optimizes brightness changes, hence it is
      // appropriate for sensors without exposure time but with a calibrated vignette.
      printf("PHOTOMETRIC MODE WITH CALIBRATION, BUT NO OR INACCURATE EXPOSURE!\n");
      settings->affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
      settings->affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    }
    return;
  }

  if (1 == sscanf(arg, "settingsFile=%s", buf))
  {
    YAML::Node settings = YAML::LoadFile(buf);
    settingsUtil.tryReadFromYaml(settings);
    printf("Loading settings from yaml file: %s!\n", buf);
    return;
  }

  if (settingsUtil.tryReadFromCommandLine(arg))
  {
    return;
  }

  printf("could not parse argument \"%s\"!!!!\n", arg);
  assert(0);
}

void MainSettings::registerArgs(SettingsUtil &set)
{
  set.registerArg("vignette", vignette);
  set.registerArg("gamma", gammaCalib);
  set.registerArg("calib", calib);
  set.registerArg("imuCalib", imuCalibFile);
  set.registerArg("speed", playbackSpeed);
  set.registerArg("preload", preload);

  set.registerArg("multiCameraIndex", settings->multiCameraIndex);

  // We don't register preset and mode as they will be handled in parseArgument.

  // Register global settings.
  set.registerArg("minOptIterations", settings->minOptIterations);
  set.registerArg("maxOptIterations", settings->maxOptIterations);
  set.registerArg("minIdepth", settings->minIdepth);
  set.registerArg("solverMode", settings->solverMode);
  set.registerArg("weightZeroPriorDSOInitY", settings->weightZeroPriorDSOInitY);
  set.registerArg("weightZeroPriorDSOInitX", settings->weightZeroPriorDSOInitX);
  set.registerArg("forceNoKFTranslationThresh", settings->forceNoKFTranslationThresh);
  set.registerArg("minFramesBetweenKeyframes", settings->minFramesBetweenKeyframes);
}

void dmvio::MainSettings::settingsDefault(int preset)
{
  printf("\n=============== PRESET Settings: ===============\n");
  if (preset == 0 || preset == 1)
  {
    printf("DEFAULT settings:\n"
           "- %s real-time enforcing\n"
           "- 2000 active points\n"
           "- 5-7 active frames\n"
           "- 1-6 LM iteration each KF\n"
           "- original image resolution\n",
           preset == 0 ? "no " : "1x");

    playbackSpeed = (preset == 0 ? 0 : 1.0);
    preload = preset == 1;
    settings->desiredImmatureDensity = 1500;
    settings->desiredPointDensity = 1000;
    settings->minFrames = 5;
    settings->maxFrames = 7;
    settings->maxOptIterations = 6;
    settings->minOptIterations = 1;

    settings->logStuff = false;
  }

  if (preset == 2 || preset == 3)
  {
    // Note: These presets were not tested with DM-VIO yet, you will probably need to adjust benchmark_width
    // and benchmark_height at least.
    printf("FAST settings:\n"
           "- %s real-time enforcing\n"
           "- 800 active points\n"
           "- 4-6 active frames\n"
           "- 1-4 LM iteration each KF\n"
           "- 424 x 320 image resolution\n",
           preset == 0 ? "no " : "5x");

    playbackSpeed = (preset == 2 ? 0 : 5);
    preload = preset == 3;
    settings->desiredImmatureDensity = 600;
    settings->desiredPointDensity = 800;
    settings->minFrames = 4;
    settings->maxFrames = 6;
    settings->maxOptIterations = 4;
    settings->minOptIterations = 1;

    settings->benchmark_width = 424;
    settings->benchmark_height = 320;

    settings->logStuff = false;
  }

  printf("==============================================\n");
}
