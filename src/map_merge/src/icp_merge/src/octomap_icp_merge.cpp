//This node reads data fom two point cloud sources - taken as octomap - voxel centers 
// and merge them using icp algorithm  

#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "octomap/ColorOcTree.h"
#include "octomap/OcTreeNode.h"
#include <string>

using namespace octomap; 
using namespace std; 

double resolution = 0.04; 
const int robot_count = 2;  
static int seq_num = 1; 
static bool completeness[robot_count];  

float floor_z = -0.14f;       //To remove the floor values


ColorOcTree tree_final(resolution);

//The temporary tree variables, which are cleared after every subscription 
AbstractOcTree* tree;
ColorOcTree* color_octree; 

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source;    // the converted cloud source 
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target;    //the converted cloud target 
 
//The Subscriber Publisher Class for the Map Merge Process ---------------------------------------------
//------------------------------------------------------------------------------------------------------
class SubscribeAndPublish{
public:
    SubscribeAndPublish(){
        pub = nh.advertise<sensor_msgs::PointCloud2>("octomap_merged",1, true);
        sub1 = nh.subscribe("/qbot1/octomap_point_cloud_centers", 1000, &SubscribeAndPublish::voxel_callback_1, this);
        sub2 = nh.subscribe("/qbot2/octomap_point_cloud_centers", 1000, &SubscribeAndPublish::voxel_callback_2, this);
    }

    void voxel_callback_1(const sensor_msgs::PointCloud2::ConstPtr& msg){
        cout<<"OCtomap centers message of robot 1 received"<<endl; 
        sensor_msgs::PointCloud2 ros_pointcloud_msg = *msg;
        //Convert into pcl format 
        pcl::PCLPointCloud2 pcl_cloud2; 
        pcl_conversions::toPCL(ros_pointcloud_msg, pcl_cloud2); 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //pcd format of point cloud
        pcl::fromPCLPointCloud2(pcl_cloud2, *pcl_cloud);
        completeness[0] = true; 
        //assign the temporaty pcl_cloud point cloud into static variable? As source 
        *cloud_source = *pcl_cloud; 
    }

    void voxel_callback_2(const sensor_msgs::PointCloud2::ConstPtr& msg){
        cout<<"OCtomap centers message of robot 1 received"<<endl; 
        sensor_msgs::PointCloud2 ros_pointcloud_msg = *msg;
        //Convert into pcl format 
        pcl::PCLPointCloud2 pcl_cloud2; 
        pcl_conversions::toPCL(ros_pointcloud_msg, pcl_cloud2); 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //pcd format of point cloud
        pcl::fromPCLPointCloud2(pcl_cloud2, *pcl_cloud);

        //assign the temporaty pcl_cloud point cloud into static variable? as target 
        *cloud_target = *pcl_cloud; 
        completeness[1] = true;
    }
    //Alighn considering two robots present 
    void icp_align_voxels() {   
        //Align cloud_source and cloud_target 
        //check whether both source and target are assigned values before applying icp. 
        if(completeness[0] and completeness[1]) {
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputSource(cloud_source);
            icp.setInputTarget(cloud_target); 
            pcl::PointCloud<pcl::PointXYZRGB> Final;
            icp.align(Final);
            std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
        }
    }


/*
        octomap_msgs::Octomap received_msg = *msg; 
        tree = octomap_msgs::fullMsgToMap(received_msg); 
        color_octree = dynamic_cast<ColorOcTree*>(tree);
        color_octree->expand(); 
        
        for(ColorOcTree::leaf_iterator it = color_octree->begin_leafs(), end=color_octree->end_leafs(); it!=end; ++it){
            
            float x = it.getX() + x_translation_1; 
            float y = it.getY() + y_translation_1; 
            float z = it.getZ();

            float sinValue = sin(yaw_1*PI/180);
            float cosValue = cos(yaw_1*PI/180); 

            float x_transformed = x*cosValue - y*sinValue; 
            float y_transformed = x*sinValue + y*cosValue; 

            if(z>floor_z){
                point3d node(x_transformed, y_transformed, z);
            
                if(it->getValue()>0.0){
                    tree_final.updateNode(node, true);
                    ColorOcTreeNode* key = tree_final.search(x_transformed, y_transformed, z);
                    key->setColor(it->getColor());

                }else{
                    tree_final.updateNode(node, false);
                    
                }
            }
        }
        completeness[0] = true;     // Mark the map1 as complete 
        bool complete = true; 
        for(int i=0; i<robot_count; i++){
            if(!completeness[i]){
                complete = false; 
                break;
            }
        }
    
        color_octree->clear(); 
        tree->clear(); 

        //Publish data as live Octomap stream 
        octomap_msgs::Octomap fmap_msg;
        octomap_msgs::fullMapToMsg(tree_final, fmap_msg);  

        fmap_msg.header.seq = seq_num;  seq_num+=1; 
        fmap_msg.header.stamp = ros::Time::now();
        fmap_msg.header.frame_id = "world"; 
        pub.publish(fmap_msg);

        if(complete){
            tree_final.write("complete.ot");
            cout << "Check the complete map at: complete.ot"<<endl;  
            
            //tree_final.clear(); 
            
            for(int j=0; j<robot_count; j++){
                completeness[j] = false; 
            }
        
        }else{
            tree_final.write("partial.ot");
            cout<<"check the partial map at: partial.ot"<<endl;
        } 
        
    }
*/
   /* void octomap_callback_2(const octomap_msgs::Octomap::ConstPtr& msg){
        cout<<"Octomap message of robot 2 received"<<endl; 

        octomap_msgs::Octomap received_msg = *msg; 
        tree = octomap_msgs::fullMsgToMap(received_msg); 
        color_octree = dynamic_cast<ColorOcTree*>(tree);
        color_octree->expand(); 
        
        for(ColorOcTree::leaf_iterator it = color_octree->begin_leafs(), end=color_octree->end_leafs(); it!=end; ++it){
            
            float x = it.getX() + x_translation_2; 
            float y = it.getY() + y_translation_2; 
            float z = it.getZ();

            float sinValue = sin(yaw_2*PI/180);
            float cosValue = cos(yaw_2*PI/180); 

            float x_transformed = x*cosValue - y*sinValue; 
            float y_transformed = x*sinValue + y*cosValue; 

            if(z>floor_z){
                point3d node(x_transformed, y_transformed, z);
            
                if(it->getValue()>0.0){
                    tree_final.updateNode(node, true);
                    ColorOcTreeNode* key = tree_final.search(x_transformed, y_transformed, z);
                    key->setColor(it->getColor());

                }else{
                    tree_final.updateNode(node, false);
                    
                }
            }
        }
        completeness[1] = true;     // Mark the map2 as complete 
        bool complete = true; 
        for(int i=0; i<robot_count; i++){
            if(!completeness[i]){
                complete = false; 
                break;
            }
        }
    
        color_octree->clear(); 
        tree->clear(); 

        //Publish data as live Octomap stream 
        octomap_msgs::Octomap fmap_msg;
        octomap_msgs::fullMapToMsg(tree_final, fmap_msg);  

        fmap_msg.header.seq = seq_num;  seq_num+=1; 
        fmap_msg.header.stamp = ros::Time::now();
        fmap_msg.header.frame_id = "world"; 
        pub.publish(fmap_msg);

        if(complete){
            tree_final.write("complete.ot");
            cout << "Check the complete map at: complete.ot"<<endl;  
            
            //tree_final.clear(); 
            
            for(int j=0; j<robot_count; j++){
                completeness[j] = false; 
            }
        
        }else{
            tree_final.write("partial.ot");
            cout<<"check the partial map at: partial.ot"<<endl;
        }
    }
*/
    private:
    ros::NodeHandle nh; 
    ros::Publisher pub;
    ros::Subscriber sub1;
    ros::Subscriber sub2; 

};


int main(int argc, char **argv){
    for(int i=0; i<robot_count; i++){
        completeness[i] = false; 
    }
    ros::init(argc, argv, "color_octomap_sub");
    SubscribeAndPublish sap; 
    sap.icp_align_voxels(); 
    ros::spin(); 
    return 0; 
}
