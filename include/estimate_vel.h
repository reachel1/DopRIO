#include <string>
#include <fstream>
#include <functional>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include <random>
#include <algorithm>

#include "radarpcl_view.h"

namespace rise{

    class Estimate_Vel{
    
    public:

        Estimate_Vel(std::string output_path){SetVelOutput(output_path);}

        virtual ~Estimate_Vel(){}

        void velocity_estimate(double time, pcl::PointCloud<RadarPointCloudType>::Ptr &raw, Eigen::Vector3d& v_radar, Eigen::Vector3d& sigma_v_radar);

        void velocity_estimate_Ransac(pcl::PointCloud<RadarPointCloudType>::Ptr &raw, pcl::PointCloud<RadarPointCloudType>::Ptr &output, 
                    Eigen::Vector3d& v_radar_inliner, Eigen::Vector3d& sigma_v_radar_inliner);

    protected:

        void SetVelOutput(std::string path_traj){

            std::string file_vel = path_traj + "/ego_vel.txt";
            if (boost::filesystem::exists(file_vel)) {
                 printf("Vel file exists, deleting old file....\n");
                 boost::filesystem::remove(file_vel);
            }
            std::cout<<file_vel<<std::endl;
            printf("%s\n",file_vel.c_str());
            out_risevel.open(file_vel.c_str());
            if (out_risevel.fail()) {
                printf("Unable to open output file %s\n",file_vel.c_str());
                std::exit(EXIT_FAILURE);
            }
        };

        /// Initial inner point set threshold
        double init_inliner_thd = 0.1;

        /// Output file stream
        std::ofstream out_risevel, out_risevel_ransac;

    };
}