
#include "FileYaml.h"


static void write(FileStorage& fs, const std::string&, const Camera& x) 
{
     x.write(fs);
}

static void read(const FileNode& node, Camera& x, const Camera& default_value = Camera()) 
{
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}

void readConfig(const string yamlPath,Camera_Other_Parameter &vecCamerasOtherParam)
{
    std::vector<Camera> vecCameras;
    cv::FileStorage fin(yamlPath, cv::FileStorage::READ);
    if(!fin.isOpened())
    {
	cerr << endl << "Failed to load readConfig yamlPath " << endl;
	return ;
    }
      
    cv::FileNode cameras_node = fin["cameras"];
/*    
    cv::FileNode Rl_node = fin["Rl"];
    cv::FileNode Rr_node = fin["Rr"];
    cv::FileNode Pl_node = fin["Pl"];
    cv::FileNode Pr_node = fin["Pr"];
    cv::FileNode Kl_node = fin["Kl"];
    cv::FileNode Kr_node = fin["Kr"];
    cv::FileNode Dl_node = fin["Dl"];
    cv::FileNode Dr_node = fin["Dr"];*/
    
    fin["Rl"] >> vecCamerasOtherParam.R_l;
    fin["Rr"] >> vecCamerasOtherParam.R_r;
    fin["Pl"] >> vecCamerasOtherParam.P_l;
    fin["Pr"] >> vecCamerasOtherParam.P_r;
    fin["Kl"] >> vecCamerasOtherParam.K_l;
    fin["Kr"] >> vecCamerasOtherParam.K_r;
    fin["Dl"] >> vecCamerasOtherParam.D_l;
    fin["Dr"] >> vecCamerasOtherParam.D_r;

 /*
    vecCamerasOtherParam.R_l = Rl_node.mat();
    vecCamerasOtherParam.R_r = Rr_node.mat();
    vecCamerasOtherParam.P_l = Pl_node.mat();
    vecCamerasOtherParam.P_r = Pr_node.mat();
    vecCamerasOtherParam.K_l = Kl_node.mat();
    vecCamerasOtherParam.K_r = Kr_node.mat();
    vecCamerasOtherParam.D_l = Dl_node.mat();
    vecCamerasOtherParam.D_r = Dr_node.mat();*/

    for (cv::FileNodeIterator it = cameras_node.begin(); it != cameras_node.end(); it++)
    {
        Camera camera;
        (*it) >> camera;
        vecCameras.push_back(camera);
    }
    
    //obtain col & row
    vecCamerasOtherParam.cols = vecCameras[0].imageDimension(0);
    vecCamerasOtherParam.rows = vecCameras[0].imageDimension(1);
    //obtain R & t
    //Eigen::Matrix4d 
    SE3 T_SC_l(vecCameras[0].T_SC.topLeftCorner(3,3),vecCameras[0].T_SC.topRightCorner(3,1));
    SE3 T_SC_r(vecCameras[1].T_SC.topLeftCorner(3,3),vecCameras[1].T_SC.topRightCorner(3,1));
    SE3 Tcl_cr = T_SC_l.inverse()*T_SC_r;
    SE3 Tcr_cl = T_SC_r.inverse()*T_SC_l;
    Matrix3d R = Tcr_cl.rotation_matrix();
    Vector3d t = Tcr_cl.translation();
    
    //Eigen tranfer to array
    double * R_ptr= new double[R.size()];
    double *t_ptr = new double[t.size()];
    Map<MatrixXd>(R_ptr, R.rows(), R.cols()) = R;
    Map<MatrixXd>(t_ptr, t.rows(), t.cols()) = t;
    cout<<"R_matrix"<<endl;
    double R_matrix[3][3];
    for(int i = 0;i < 3;i++)
      for(int j = 0;j<3;j++)
      {
	  //transpose
	  R_matrix[j][i] = R_ptr[i+j*3];
	  cout<<R_matrix[j][i]<<endl;
      }
       
    cout<<"t_matrix"<<endl;
    double t_matrix[3];
    for(int i = 0;i < 3;i++)
    {
	t_matrix[i] = t_ptr[i];
	cout<<t_matrix[i]<<endl;
    }
    vecCamerasOtherParam.R = cv::Mat(3,3,CV_64FC1,R_matrix);
    vecCamerasOtherParam.t = cv::Mat(3,1,CV_64FC1,t_matrix);
}
void Camera_refishe_Parameter::rm_fisheye(const string& output_yaml)
{
    
    //-----------------fisheye rectify---------------------------------------------
    cv::Mat Q;
    
    if(vecCamerasParam.K_l.empty() || vecCamerasParam.K_r.empty() || vecCamerasParam.P_l.empty() || vecCamerasParam.P_r.empty() || vecCamerasParam.R_l.empty() || 
	    vecCamerasParam.R_r.empty() || vecCamerasParam.D_l.empty() || vecCamerasParam.D_r.empty() || vecCamerasParam.rows==0 || vecCamerasParam.cols==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return ;
    }
    
    cv::fisheye::initUndistortRectifyMap(vecCamerasParam.K_l,vecCamerasParam.D_l,vecCamerasParam.R_l,vecCamerasParam.P_l.rowRange(0,3).colRange(0,3),
					 cv::Size(vecCamerasParam.cols,vecCamerasParam.rows),CV_32FC1,M1l,M2l);
    cv::fisheye::initUndistortRectifyMap(vecCamerasParam.K_r,vecCamerasParam.D_r,vecCamerasParam.R_r,vecCamerasParam.P_r.rowRange(0,3).colRange(0,3),
					 cv::Size(vecCamerasParam.cols,vecCamerasParam.rows),CV_32FC1,M1r,M2r);
    cout << "the P_l of initUndistortRectifyMap after" << endl;
    for(int i = 0;i < 3;++i)
      for(int j = 0;j < 3;++j)
      {
	  double *ptr = vecCamerasParam.P_l.ptr<double>(i,j);
	  cout << *ptr<<endl;
      }
    cout << "the P_r of initUndistortRectifyMap after" << endl;
    for(int i = 0;i < 3;++i)
      for(int j = 0;j < 3;++j)
      {
	  double *ptr = vecCamerasParam.P_r.ptr<double>(i,j);
	  cout << *ptr<<endl;
      }
    cv::FileStorage fs(output_yaml, FileStorage::WRITE);

    fs << "fx" << vecCamerasParam.P_l.at<double>(0,0);
    fs << "fy" << vecCamerasParam.P_l.at<double>(1,1);
    fs << "cx" << vecCamerasParam.P_l.at<double>(0,2);
    fs << "cy" << vecCamerasParam.P_l.at<double>(1,2);

    fs << "k1" << 0.0;
    fs << "k2" << 0.0;
    fs << "p1" << 0.0;
    fs << "p2" << 0.0;
    
    fs << "width" << vecCamerasParam.cols;
    fs << "height" << vecCamerasParam.rows;

    fs.release();
    cout << "the P_r of initUndistortRectifyMap after" << endl;
    for(int i = 0;i < 3;++i)
      for(int j = 0;j < 3;++j)
      {
	  double *ptr = vecCamerasParam.P_r.ptr<double>(i,j);
	  cout << *ptr<<endl;
      }
    cv::stereoRectify(vecCamerasParam.K_l,vecCamerasParam.D_l,vecCamerasParam.K_r,vecCamerasParam.D_r,cv::Size(vecCamerasParam.cols,vecCamerasParam.rows),
		      vecCamerasParam.R,vecCamerasParam.t,vecCamerasParam.R_l,vecCamerasParam.R_r,vecCamerasParam.P_l,vecCamerasParam.P_r,Q,cv::CALIB_ZERO_DISPARITY,0);

    //-----------------fisheye end---------------------------------------------
}