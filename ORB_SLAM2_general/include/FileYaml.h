#ifndef _FILEYAML_H
#define _FILEYAML_H


#include <iostream>
#include <fstream>
using namespace std;

#include <opencv/cv.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include<opencv2/opencv.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <sophus/so3.h>
#include <sophus/se3.h>
#include <chrono>
using namespace cv;
using namespace Eigen;
using namespace Sophus;

using cv::FileStorage;
using cv::FileNode;
struct Camera
{
    Eigen::Matrix4d T_SC;
    Eigen::Vector2d imageDimension;
    Eigen::VectorXd distortionCoefficients;
    Eigen::Vector2d focalLength;
    Eigen::Vector2d principalPoint;
    std::string distortionType;

    Camera() :
            T_SC(Eigen::Matrix4d::Identity()),
            imageDimension(Eigen::Vector2d(1280, 800)),
            distortionCoefficients(Eigen::Vector2d(0, 0)),
            focalLength(Eigen::Vector2d(700, 700)),
            principalPoint(Eigen::Vector2d(640, 400)),
            distortionType("equidistant")
    {

    }

    void write(FileStorage& fs) const //Write serialization for this class
    {
        fs << "{:";

        fs << "T_SC";
        fs << "[:";
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                fs << T_SC(i, j);
            }
        }
        fs << "]";
        fs << "image_dimension";
        fs << "[:";
        fs << imageDimension(0) << imageDimension(1);
        fs << "]";

        fs << "distortion_coefficients";
        fs << "[:";
        for (int i = 0; i < distortionCoefficients.rows(); i++)
        {
            fs << distortionCoefficients(i);
        }
        fs << "]";

        fs << "distortion_type" << distortionType;

        fs << "focal_length";
        fs << "[:";
        fs << focalLength(0) << focalLength(1);
        fs << "]";

        fs << "principal_point";
        fs << "[:";
        fs << principalPoint(0) << principalPoint(1);
        fs << "]";

        fs << "}";
    }

    void read(const FileNode& node)  //Read serialization for this class
    {
        cv::FileNode T_SC_node = node["T_SC"];
        cv::FileNode imageDimensionNode = node["image_dimension"];
        cv::FileNode distortionCoefficientNode = node["distortion_coefficients"];
        cv::FileNode focalLengthNode = node["focal_length"];
        cv::FileNode principalPointNode = node["principal_point"];

        // extrinsics
        T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3], T_SC_node[4], T_SC_node[5], T_SC_node[6], T_SC_node[7], T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11], T_SC_node[12], T_SC_node[13], T_SC_node[14], T_SC_node[15];

        imageDimension << imageDimensionNode[0], imageDimensionNode[1];
        distortionCoefficients.resize(distortionCoefficientNode.size());
        for (size_t i = 0; i<distortionCoefficientNode.size(); ++i) 
	{
            distortionCoefficients[i] = distortionCoefficientNode[i];
        }
        focalLength << focalLengthNode[0], focalLengthNode[1];
        principalPoint << principalPointNode[0], principalPointNode[1];
        distortionType = (std::string)(node["distortion_type"]);
    }
};

struct Camera_Other_Parameter
{
    Mat K_l;
    Mat K_r;
    Mat D_l;
    Mat D_r;
    Mat R_l;
    Mat R_r;
    Mat P_l;
    Mat P_r;
    Mat R;
    Mat t;
    int cols;		//图像宽
    int rows;		//图像高
};
struct Camera_refishe_Parameter
{
    Camera_Other_Parameter vecCamerasParam;
    cv::Mat M1l,M2l,M1r,M2r;
    void rm_fisheye(const string& output_yaml);
};
void readConfig(const string yamlPath,Camera_Other_Parameter &vecCamerasOtherParam);

#endif