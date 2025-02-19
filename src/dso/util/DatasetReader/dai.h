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

#include "util/DatasetReader.h"
#include <filesystem>

using namespace dso;

class DaiFolderReader : public DatasetReader
{
public:
  DaiFolderReader(std::string path, std::string calibFile, bool use16BitPassed)
  {
    this->path = path;
    this->calibfile = calibFile;
    use16Bit = use16BitPassed;

    loadImages();

    undistort = Undistort::makeFromCalibration(calibFile);

    widthOrg = undistort->getOriginalSize()[0];
    heightOrg = undistort->getOriginalSize()[1];
    width = undistort->getSize()[0];
    height = undistort->getSize()[1];

    printf("DaiFolderReader: got %d files in %s!\n", (int)files[multiCameraIndex].size(), path.c_str());
  }

  Eigen::VectorXf getOriginalCalib()
  {
    return undistort->getOriginalParameter().cast<float>();
  }
  Eigen::Vector2i getOriginalDimensions()
  {
    return undistort->getOriginalSize();
  }

  void getCalibMono(Eigen::Matrix3f &K, int &w, int &h)
  {
    K = undistort->getK().cast<float>();
    w = undistort->getSize()[0];
    h = undistort->getSize()[1];
  }

  void setGlobalCalibration()
  {
    int w_out, h_out;
    Eigen::Matrix3f K;
    getCalibMono(K, w_out, h_out);
    setGlobalCalib(w_out, h_out, K);
  }

  int getNumImages()
  {
    return files[multiCameraIndex].size();
  }

  double getTimestamp(int id)
  {
    // TODO adapt for more cameras
    if (timestamps[multiCameraIndex].size() == 0)
      return id * 0.1f;
    if (id >= (int)timestamps[multiCameraIndex].size())
      return 0;
    if (id < 0)
      return 0;
    return timestamps[multiCameraIndex][id];
  }

  std::string getFilename(int id)
  {
    // TODO adapt for more cameras
    return files[multiCameraIndex][id];
  }

  void prepImage(int id, bool as8U = false)
  {
  }

  MinimalImageB *getImageRaw(int id)
  {
    return getImageRaw_internal(id, 0);
  }

  ImageAndExposure *getImage(int id, bool forceLoadDirectly = false)
  {
    return getImage_internal(id, 0);
  }

  inline float *getPhotometricGamma()
  {
    if (undistort == 0 || undistort->photometricUndist == 0)
      return 0;
    return undistort->photometricUndist->getG();
  }

  dmvio::IMUData getIMUData(int i)
  {
    // returning IMU data between frame i-1 and frame i!
    return imuDataAllFrames[i - 1];
  }

  dmvio::GTData getGTData(int id, bool &foundOut)
  {
    // TODO
    foundOut = false;
    return dmvio::GTData();
  }

  bool loadGTData(std::string gtFile)
  {
    // TODO
    return false;
  }

  void loadIMUData(std::string imuFile = "")
  {
    // Important: This IMU loading method expects that for each image there is an IMU 'measurement' with exactly the same timestamp (the VI-sensor does this).
    // If the sensor does not output this, a fake measurement with this timestamp has to be interpolated in advance.
    // The DM-VIO Python tools have a script to do this.
    if (imuFile == "")
    {
      imuFile = path / "imu.csv";
    }
    std::ifstream imuStream(imuFile);
    if (imuStream.good())
    {
      std::string line;
      std::getline(imuStream, line);
      // At the moment only comments at the beginning of the file are supported.
      while (line[0] == '#')
      {
        std::cout << "Skipping comment line in IMU data.\n";
        std::getline(imuStream, line);
      }
      std::stringstream lineStream(line);
      char tmp;
      long long imuStamp;
      double wx, wy, wz, ax, ay, az;
      lineStream >> imuStamp >> tmp >> wx >> tmp >> wy >> tmp >> wz >> tmp >> ax >> tmp >> ay >> tmp >> az;
      std::cout << "IMU Id: " << imuStamp << std::endl;

      // Find first frame with IMU data.
      int startFrame = -1;
      for (size_t j = 0; j < getNumImages(); ++j)
      {
        long long imageTimestamp = ids[j];
        while (imuStamp < imageTimestamp)
        {
          imuStream >> imuStamp >> tmp >> wx >> tmp >> wy >> tmp >> wz >> tmp >> ax >> tmp >> ay >> tmp >> az;
        }
        if (imuStamp == imageTimestamp)
        {
          // Success
          startFrame = j;
          break;
        }
        if (imuStamp > imageTimestamp)
        {
          std::cout << "IMU-data too old -> skipping frame" << std::endl;
          imuDataAllFrames.push_back(dmvio::IMUData{});
          continue;
        }
      }

      if (startFrame == -1)
      {
        std::cout << "Found no start frame for IMU-data!" << std::endl;
        imuStream.close();
        return;
      }

      // For each image, we will save the IMU data between it, and the next frame, so no IMU data is needed for the last frame.
      // Note that when later accessing the imu data in the method getIMUData we output the imu data between the given frame and the previous frame.
      for (size_t j = startFrame; j < getNumImages() - 1; j++)
      {
        long long imageTimestamp = ids[j];
        long long nextTimestamp = ids[j + 1];

        assert(imuStamp == imageTimestamp); // Otherwise we would need to interpolate IMU data which is not implemented atm.
        dmvio::IMUData imuData;
        long long previousIMUTime = imuStamp;

        // Each frame should get the all IMU data with:
        // thisTimestamp < imuStamp <= nextTimestamp
        while (imuStamp < nextTimestamp)
        {
          // Get next IMU-Data.
          imuStream >> imuStamp >> tmp >> wx >> tmp >> wy >> tmp >> wz >> tmp >> ax >> tmp >> ay >> tmp >> az;

          if (imuStamp > nextTimestamp)
          {
            // If this happens we would have to interpolate IMU data which is not implemented at the moment.
            assert(false);
          }

          Eigen::Vector3d accMeas, gyrMeas;
          accMeas << ax, ay, az;
          gyrMeas << wx, wy, wz;
          // For each measurement GTSAM wants the time between it, and the previous measurement.
          // The timestamps are in nanoseconds -> convert!
          double integrationTime = (double)(imuStamp - previousIMUTime) * 1e-9;
          imuData.push_back(dmvio::IMUMeasurement(accMeas, gyrMeas, integrationTime));

          previousIMUTime = imuStamp;
        }

        imuDataAllFrames.push_back(imuData);
      }
    }
    else
    {
      std::cout << "Found no IMU-data." << std::endl;
    }

    imuStream.close();
  }

private:
  MinimalImageB *getImageRaw_internal(int id, int unused)
  {
    assert(!use16Bit);
    // TODO adapt for more cameras
    return IOWrap::readImageBW_8U(files[multiCameraIndex][id]);
  }

  ImageAndExposure *getImage_internal(int id, int unused)
  {
    // TODO adapt for more cameras
    if (use16Bit)
    {
      MinimalImage<unsigned short> *minimg = IOWrap::readImageBW_16U(files[multiCameraIndex][id]);
      assert(minimg);
      ImageAndExposure *ret2 = undistort->undistort<unsigned short>(
          minimg,
          (exposures.size() == 0 ? 1.0f : exposures[multiCameraIndex][id]),
          (timestamps.size() == 0 ? 0.0 : timestamps[multiCameraIndex][id]),
          1.0f / 256.0f);
      delete minimg;
      return ret2;
    }
    else
    {
      MinimalImageB *minimg = getImageRaw_internal(id, 0);
      ImageAndExposure *ret2 = undistort->undistort<unsigned char>(
          minimg,
          (exposures.size() == 0 ? 1.0f : exposures[multiCameraIndex][id]),
          (timestamps.size() == 0 ? 0.0 : timestamps[multiCameraIndex][id]));
      delete minimg;
      return ret2;
    }
  }

  void loadImages()
  {
    files.clear();
    timestamps.clear();
    exposures.clear();

    bool exposures_bad = false;

    const auto cams_path = path / "cams";
    std::vector<std::filesystem::path> cams_meta;
    for (const auto entry : std::filesystem::directory_iterator(cams_path))
    {
      const auto entry_path = entry.path();
      if (entry_path.extension() == ".csv")
      {
        cams_meta.push_back(entry_path);
      }
    }
    std::sort(cams_meta.begin(), cams_meta.end());
    for (const auto &meta : cams_meta)
    {
      const auto cam_dir = cams_path / meta.stem();
      auto &cam_files = files.emplace_back();
      auto &cam_timestamps = timestamps.emplace_back();
      auto &cam_exposures = exposures.emplace_back();
      ids.clear();

      std::ifstream f(meta);
      std::string line;
      while (std::getline(f, line))
      {
        if (line[0] == '#')
          continue;

        std::stringstream ss(line);

        char tmp;
        std::string filename;
        int64_t timestamp;
        int64_t exposure;

        ss >> timestamp >> tmp >> exposure >> tmp >> filename;

        if (exposure == 0)
          exposures_bad = true;

        cam_files.push_back(cam_dir / filename);
        cam_timestamps.push_back(timestamp * 1e-9);
        cam_exposures.push_back(exposure * 1e-6);
        ids.push_back(timestamp);
      }
    }

    if (exposures_bad)
    {
      printf("set EXPOSURES to zero!\n");
      exposures.clear();
    }

    printf("got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps[multiCameraIndex].size(), (int)exposures[multiCameraIndex].size());
  }

  std::map<long long, dmvio::GTData> gtData;

  std::vector<ImageAndExposure *> preloadedImages;
  std::vector<std::vector<std::string>> files;
  std::vector<std::vector<double>> timestamps;
  std::vector<std::vector<float>> exposures;
  std::vector<long long> ids; // Saves the ids that are used by e.g. the EuRoC dataset.

  std::vector<dmvio::IMUData> imuDataAllFrames;

  int width, height;
  int widthOrg, heightOrg;

  std::filesystem::path path;
  std::filesystem::path calibfile;

  bool use16Bit;
};
