#include <iostream>
#include "ros/ros.h"
#include <camera/MarkerPose.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>

using namespace std;
using namespace Eigen;

// Marker pose data
double Marker_x,Marker_y,Marker_z;
Eigen::Vector4d qFromMarker;

Eigen::Matrix4d T_TC;
Eigen::Matrix4d T_CT;   // Inverse of T_TC--> Allow the transformation between Camera and Tool
Eigen::Matrix4d T_CC;   // Calibration matrix Camera
Eigen::Matrix4d T_CQRC;
ros::Publisher pub_Kukapos;


Vector4d R2quaternion(const Eigen::Matrix3d& R)
{

  Vector4d q;
  q(0)=0.5*sqrt(1+R(0,0)+R(1,1)+R(2,2));
  q(1)=1/(4*q(0))*(R(2,1)-R(1,2));
  q(2)=1/(4*q(0))*(R(0,2)-R(2,0));
  q(3)=1/(4*q(0))*(R(1,0)-R(0,1));

  return q;
}

// Callback to read the position of the end effector of the Kuka and compute the haptic force
void KukaPosCallback(const geometry_msgs::Pose &msg){

    Matrix3d R;
    geometry_msgs::Pose KukaWS_pose;
    double den=1+pow(msg.orientation.x,2)+pow(msg.orientation.y,2)+pow(msg.orientation.z,2);
    double v1=msg.orientation.x;
    double v2=msg.orientation.y;
    double v3=msg.orientation.z;
    double v4=msg.orientation.w;    

    /*
    // Rotation matrix from Rodrigues vector
    R << ((pow(v1,2)-pow(v2,2)-pow(v3,2)+pow(v4,2))),(2*(v1*v2-v3*v4)),(2*(v1*v3+v2*v4)),
    (2*(v1*v2+v3*v4)),((-pow(v1,2)+pow(v2,2)-pow(v3,2)+pow(v4,2))),(2*(v3*v2-v1*v4)),
    (2*(v1*v3-v2*v4)),(2*(v3*v2+v1*v4)),((-pow(v1,2)-pow(v2,2)+pow(v3,2)+pow(v4,2)));

    R = R/den;    
    */

   cv::Mat v = ( cv::Mat_<double>(3, 1)<<v1,v2,v3);

   cv::Mat R_cv(3, 3, CV_32F);

   cv::Rodrigues(v, R_cv);

   cv::cv2eigen(R_cv, R);

   Matrix3d R_transp = R.transpose();

    //cout<<"R: "<< R << endl;
     //cout<<"R_T: "<< R_transp << endl;

//     //qMarker= aa2quaternion(aa);
//     //cout<<"R: "<<R<<endl;

    T_CQRC=Eigen::Matrix4d::Identity();
//     T_CC=Eigen::Matrix4d::Identity();
    
//     //Get the Marker pose and compute the workspace center to send to the kuka

    Marker_x = msg.position.x;
    Marker_y = msg.position.y;
    Marker_z = msg.position.z;

    int i,j;

    for(i=0;i<=2;i++){
        for(j=0;j<=2;j++){
            T_CQRC(i,j)=R_transp(i,j);
            // T_CQRC(i,j)=R(i,j);
        }
    }

    T_CQRC(0,3)=Marker_x;
    T_CQRC(1,3)=Marker_y;
    T_CQRC(2,3)=Marker_z;

//     T_CC(0,3)=Marker_x;
//     T_CC(1,3)=Marker_y;
//     T_CC(2,3)=Marker_z;

//     MatrixXd CornerQRcode(4, 5); 
//     CornerQRcode << 0, -0.025, 0.025, 0.025, -0.025,
//     0, 0.025, 0.025, -0.025, -0.025,
//     0, 0, 0, 0, 0,
//     1, 1, 1, 1, 1;
  
//     MatrixXd Mesh(4,5);
//     Mesh << -0.0601594, 0.02944, -0.080256, -0.091753, -0.041307,
//     -0.0201621, -0.004392, 0.002095, -0.036977, -0.042683,
//     0.187, 0.1988, 0.2083, 0.1761, 0.166,
//     1,1,1,1,1;

    std::cout << "T_CQRC"<< T_CQRC << std::endl;

//     MatrixXd QR_code(4,5);
//     QR_code = CornerQRcode.transpose()*T_CC;
//     QR_code.transposeInPlace();
//     std::cout << "QR_CODE MATRIX:"<< QR_code << std::endl;

//     Mesh.transposeInPlace();
//     QR_code.transposeInPlace();
//     Matrix4d aff = Mesh.completeOrthogonalDecomposition().solve(QR_code);
//     std::cout <<"Aff:"<<std::endl;
//     std::cout<< aff.transpose() << std::endl;
//     aff.transposeInPlace();

//     aff<<0.706035,0.506597,-0.0934815,0.0749398,
//     0.173512,0.741365,0.164309,-0.0146163,
//    -0.566432,0.599885,0.380517,0.125243,
//     0,0,0,1;


    /*
    std::vector<cv::Point3f> first, second;
    //float x[]={};
    std::vector<uchar> inliers;
    cv::Mat aff(3,4,CV_64F);   

    for (int i = 0; i <5; i++)
    {        
        first.push_back(cv::Point3f(QR_code(0,i),QR_code(1,i),QR_code(2,i)));
        second.push_back(cv::Point3f(Mesh(0,i),Mesh(1,i),Mesh(2,i)));
    }       

    int ret = cv::estimateAffine3D(first, second, aff, inliers);
    std::cout << aff << std::endl;
    */

    Matrix4d T_rgb_mesh;
    // T_rgb_mesh << 0.897215595515957,-0.21823220493857, 0.383899570057843,-0.135156848289384,
    //     0.204257787518251, 0.975855345047695, 0.0773634395563748, -0.0282380930202606,
    //     -0.391513641398478, 0.0090027923164871, 0.920128261890379, -0.00515450705919404,
    //     0,0,0,1;
    
      

    Matrix4d T_mesh_tip;
    // T_mesh_tip << 0.301991254264524, -0.743520993507364, -0.596638763877745, 0.164598457513567,
    // -0.941938658564648, -0.136342304943358, -0.306858826472639, 0.0315377569690366,
    // 0.146808875239793, 0.654665798783284, -0.741525350918141, 0.220592995248951,
    // 0,0,0,1;

    // T_mesh_tip <<-0.630219783734357, -0.76241053053431, -0.146810105647441, -0.0523847442230844, 
    // 0.639701926374248, -0.402723097233904, -0.654672095286951, -0.0177323128486233,
    // 0.440005079042159, -0.506502013702536, 0.741519548314393, 0.271459059163145,
    // 0,0,0,1;

    // T_mesh_tip << 0.069434315595335, -0.986724717789919, -0.146810105647441, -0.0523847442230844,
    // 0.740972496501286, 0.14954667191631, -0.654672095286951, -0.0177323128486233,
    // 0.667936101170208, -0.0633255416175884, 0.741519548314393, 0.271459059163145,
    // 0, 0, 0, 1;

    // T_mesh_tip << 0.485567480058694, -0.86840299753448, 0.100501025778741, -0.0680947226234418,
    // -0.404631845381959, -0.121352330240058, 0.906392123558065, -0.00178257168971064,
    // -0.7749176033697, -0.480780454904771, -0.410308252621517, 0.282633312756131,
    // 0,0,0,1;

    //T_MESH_TIP LAST
    // T_mesh_tip << 0.280930010089661, -0.913304169590609, -0.294879336745441, -0.0531812389276266,
    // 0.0618904550221919, -0.289373270211582, 0.955213422259236, -0.0509498673232298, 
    // -0.957730599390125, -0.286598332680884, -0.0247688250905902, 0.245354150048021,
    // 0,0,0,1;

    //T_MESH_TIP With offset tip and 7 joint
    // T_mesh_tip << 0.353758251076815, -0.907517003930291, -0.226424352428048, 0.0165402488070215,
    // -0.136376779679371, -0.289536429787512, 0.947401725663504, -0.0394554292304665,
    // -0.925341274211497, -0.304272153512754, -0.22619014752494, 0.287869167295127,
    // 0, 0, 0, 1;

    //T_MESH_TIP With offset tip and 7 joint with minus
    // T_mesh_tip << 0.226366292019801, -0.91257296587445, -0.340542044089004, -0.126713252925057,
    // 0.482507267442731, -0.198649496084831, 0.853068059752674 , -0.046132236926905, 
    // -0.846135354835239, -0.357419864669486, 0.395355538265864, 0.199109794072256,
    // 0, 0, 0, 1;

    T_mesh_tip << -0.679157817125201, -0.530256030810586, 0.507516700441212, -0.0634974394640862,
    0.415645161230351, -0.847740320234214, -0.329507889731017, -0.00403834817010895, 
    0.604965915885786, -0.012840998333266, 0.79614781879892, 0.235910663416293,
    0,0,0,1;

    Matrix4d T_tip_mesh = T_mesh_tip.inverse();
    std::cout<<" "<<std::endl;
    std::cout<<"T_tip_mesh"<<std::endl;
    std::cout << T_tip_mesh << std::endl;

    Matrix4d T_TQRC=Eigen::Matrix4d::Identity();
    Matrix3d Rot;
    //T_TQRC = T_CT*aff*T_CQRC;
    //T_TQRC = T_CQRC*aff*T_CT;
    T_TQRC = T_tip_mesh*T_rgb_mesh*T_CQRC;
    // T_TQRC(2,3)=T_TQRC(2,3)+0.04;
    //T_TQRC = T_CQRC*T_CT;
    Matrix4d T_Prova=Eigen::Matrix4d::Identity();
    T_Prova=T_rgb_mesh*T_CQRC;

    std::cout<<" "<<std::endl;
    std::cout<<"T_Prova"<<std::endl;
    std::cout << T_Prova << std::endl;
    // std::cout<<"T_TQRC"<<std::endl;
    // std::cout << T_TQRC << std::endl;

    //Quaternions from Markers in Tip Reference Frame
    for(i=0;i<=2;i++){
        for(j=0;j<=2;j++){
            Rot(i,j)=T_TQRC(i,j);
        }
    }
    qFromMarker = R2quaternion(Rot);

    KukaWS_pose.position.x= T_TQRC(0,3);
    KukaWS_pose.position.y= T_TQRC(1,3);
    KukaWS_pose.position.z= T_TQRC(2,3);

    KukaWS_pose.orientation.x=qFromMarker(1);
    KukaWS_pose.orientation.y=qFromMarker(2);
    KukaWS_pose.orientation.z=qFromMarker(3);
    KukaWS_pose.orientation.w=qFromMarker(0);

    pub_Kukapos.publish(KukaWS_pose);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "computation");
    ros::NodeHandle nh;
    pub_Kukapos = nh.advertise<geometry_msgs::Pose>("kuka_wsCenter", 100);
    ros::Subscriber sub_markerpos = nh.subscribe("computation/marker/position", 100, &KukaPosCallback);
    ros::Rate loop_rate(100);

    T_TC=Eigen::Matrix4d::Identity();
    T_CT=Eigen::Matrix4d::Identity();

    // T_TC(0,0) = 0;
    // T_TC(0,1) = 0;
    // T_TC(0,2) = 0;
    // T_TC(0,3) = 0;
    // T_TC(1,0) = -1;
    // T_TC(1,1) =-0.59342;
    // T_TC(1,2) =-0.8049;
    // T_TC(1,3) =0.15429933;
    // T_TC(2,0) =-1;
    // T_TC(2,1) =0;
    // T_TC(2,2) =0;
    // T_TC(2,3) =-0.07;
    // T_TC(3,0) =0;
    // T_TC(3,1) =0;
    // T_TC(3,2) =0;
    // T_TC(3,3) =1;

    // T_CT(0,0)=T_TC(0,0);
    // T_CT(0,1)=T_TC(1,0);
    // T_CT(0,2)=T_TC(2,0);
    // T_CT(0,3)=-0.08429933;
    // T_CT(1,0)=T_TC(0,1);
    // T_CT(1,1)=T_TC(1,1);
    // T_CT(1,2)=T_TC(2,1);
    // T_CT(1,3)=-0.0915643084086;
    // T_CT(2,0)=T_TC(0,2);
    // T_CT(2,1)=T_TC(1,2);
    // T_CT(2,2)=T_TC(2,2);
    // T_CT(2,3)=-0.124195530717;

    T_CT(0,0)=0.81915;
    T_CT(0,1)=0;
    T_CT(0,2)=-0.57358;
    T_CT(0,3)=-0.058403;
    T_CT(1,0)=0;
    T_CT(1,1)=1;
    T_CT(1,2)=0;
    T_CT(1,3)=-0.00284;
    T_CT(2,0)=0.57358;
    T_CT(2,1)=0;
    T_CT(2,2)=0.81915;
    T_CT(2,3)=0.2347;

    ros::spin();
}

