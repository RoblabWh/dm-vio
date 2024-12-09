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

#include <dirent.h>
#include "IMU/IMUTypes.h"
#include "util/GTData.hpp"
#include "util/Undistort.h"

using namespace dso;

struct PrepImageItem
{
  int id;
  bool isQueud;
  ImageAndExposure *pt;

  inline PrepImageItem(int _id)
  {
    id = _id;
    isQueud = false;
    pt = 0;
  }

  inline void release()
  {
    if (pt != 0)
      delete pt;
    pt = 0;
  }
};

class DatasetReader
{
public:
  virtual ~DatasetReader() = default;

  virtual Eigen::VectorXf getOriginalCalib() = 0;
  virtual Eigen::Vector2i getOriginalDimensions() = 0;

  virtual void getCalibMono(Eigen::Matrix3f &K, int &w, int &h) = 0;

  virtual void setGlobalCalibration() = 0;

  virtual int getNumImages() = 0;

  virtual double getTimestamp(int id) = 0;

  virtual std::string getFilename(int id) = 0;

  virtual void prepImage(int id, bool as8U = false) = 0;

  virtual MinimalImageB *getImageRaw(int id) = 0;

  virtual ImageAndExposure *getImage(int id, bool forceLoadDirectly = false) = 0;

  virtual inline float *getPhotometricGamma() = 0;

  virtual dmvio::IMUData getIMUData(int i) = 0;

  virtual dmvio::GTData getGTData(int id, bool &foundOut) = 0;

  virtual bool loadGTData(std::string gtFile) = 0;

  virtual void loadIMUData(std::string imuFile = "") = 0;

  // undistorter. [0] always exists, [1-2] only when MT is enabled.
  Undistort *undistort = nullptr;
};

// Implementations for DatasetReader
#include "DatasetReader/dso.h"
#include "DatasetReader/dai.h"
