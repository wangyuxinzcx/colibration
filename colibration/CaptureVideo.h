// CaptureProc.h
#ifndef CAPTUREPROC_H
#define CAPTUREPROC_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <cstdio>
#include <windows.h>


class CaptureVideo {
public:

    static std::string to_upper(const std::string& str);

    static int execute_colmap_commands(
        const std::string& colmap_path,
        const std::string& database_path,
        const std::string& image_path,
        const std::string& output_path);

    static Eigen::Matrix3d quaternionToRotationMatrix(double w, double x, double y, double z);

    //static void save_lines(const std::string& input_file_path, const std::string& output_file_path);

    //static void save_lines_containing_jpg(const std::string& images_file_path, const std::string& camera_file_path, const std::string& output_xml_path);

    static std::vector<Eigen::Matrix3d> getCameraExtrinsics(const std::string& filePath, int numCameras);

    static std::vector<Eigen::Vector3d> getCameraExtrinsics_T(const std::string& filePath, int numCameras);

    static std::vector<Eigen::Matrix3d> getCameraIntrinsics(const std::string& filePath, int numCameras);

    static std::vector<Eigen::Matrix3d> computeRectificationMatrices(
        int numCameras,
        const std::vector<Eigen::Matrix3d>& rotationMatrices,
        const std::vector<Eigen::Matrix3d>& intrinsics);

    static void saveToXML(const std::vector<Eigen::Matrix3d>& rectificationMatrices, const std::string& outputFilePath);

    static void Calibrate();

    static std::vector<Eigen::Matrix3d> computeRectificationMatrices_2(
        int numCameras,
        const std::vector<Eigen::Matrix3d>& rotationMatrices,
        const std::vector<Eigen::Matrix3d>& intrinsics,
        const std::vector<Eigen::Vector3d>& T);
};

#endif // CAPTUREPROC_H
#pragma once

#pragma once
