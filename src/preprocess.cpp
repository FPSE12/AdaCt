//
// Created by wjj on 2023/11/14.
//
#include "AdaCt/utility.h"
#include <opencv/cv.h>

class Preprocess: public configParam{
public:
    ros::Subscriber lidar_sub;

    ros::Publisher pubFullCloud;
    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubOutlierCloud;
    std_msgs::Header cloud_header;



    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIRT>::Ptr fullCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr groundCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr segmentedCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr outlierCloud;

    PointXYZIRT nanPoint;


    cv::Mat rangeMat;
    cv::Mat groundMat;
    cv::Mat labelMat;


    std::vector<int> Point_row;
    std::vector<int> Point_col;
    std::vector<double> Point_range;

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    int labelCount;
    int valid_points_num;
    std::vector<std::pair<int8_t, int8_t> > neighborIterator;

    //segment info
    std::vector<int> startRingIndex;
    std::vector<int> endRingIndex;

    std::vector<bool> segmentedCloudGroundFlag;
    std::vector<int> segmentedCloudColInd;
    std::vector<double> segmentedCloudRange;

    Preprocess(){
        lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, &Preprocess::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        groundCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        segmentedCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        outlierCloud.reset(new pcl::PointCloud<PointXYZIRT>());

        fullCloud->points.resize(N_SCAN *Horizon_SCAN);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;
        // rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
//        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
//        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
//
//        rangeMat = cv::Mat(N_SCAN,Horizon_SCAN,CV_64FC4, cv::Scalar::all(FLT_MAX));

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);


        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];


    }


    void allocateMemory(){
        laserCloudIn->clear();
        //fullCloud->clear();
        outlierCloud->clear();
        segmentedCloud->clear();
        groundCloud->clear();
        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);

        Point_range.clear();
        Point_row.clear();
        Point_col.clear();


        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        //rangeMat = cv::Mat(N_SCAN,Horizon_SCAN,CV_64FC4, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

        Point_col.reserve(N_SCAN*Horizon_SCAN);
        Point_row.reserve(N_SCAN*Horizon_SCAN);
        Point_range.reserve(N_SCAN*Horizon_SCAN);
        std::fill(Point_row.begin(), Point_row.end(), -1);
        std::fill(Point_col.begin(),Point_col.end(),-1);
        std::fill(Point_range.begin(),Point_range.end(),-1);



        labelCount=1;

        startRingIndex.clear();
        endRingIndex.clear();
        segmentedCloudRange.clear();
        segmentedCloudGroundFlag.clear();
        segmentedCloudColInd.clear();

        startRingIndex.reserve(N_SCAN);
        endRingIndex.reserve(N_SCAN);

        segmentedCloudColInd.reserve(N_SCAN * Horizon_SCAN);
        segmentedCloudGroundFlag.reserve(N_SCAN * Horizon_SCAN);
        segmentedCloudRange.reserve(N_SCAN * Horizon_SCAN);

        valid_points_num=0;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr & laserCloud){

        auto start = std::chrono::steady_clock::now();
        allocateMemory();

        cloud_header = laserCloud->header;
        pcl::fromROSMsg(*laserCloud, *laserCloudIn);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn,*laserCloudIn,indices);



        projectPointcloud();
        groundRemoval();
        cloudSegmentation();
        publish();
        auto end=std::chrono::steady_clock::now();
        //about 20ms
        ROS_INFO("INIT COST: %f ms",
                 std::chrono::duration<double, std::milli>(end - start).count());
    }


    void projectPointcloud(){
        float horizonAngle;
        size_t row,col;
        int cloud_size=laserCloudIn->size();
        PointXYZIRT point;

        for(int i=0;i<cloud_size;i++){
            point.x=laserCloudIn->points[i].x;
            point.y=laserCloudIn->points[i].y;
            point.z=laserCloudIn->points[i].z;
            row = laserCloudIn->points[i].ring;
            if(row<0 || row>N_SCAN){
                continue;
            }
            horizonAngle = atan2(point.x,point.y)*180/M_PI;
            col=-round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;

            if(col>=Horizon_SCAN){
                col -= Horizon_SCAN;
            }
            if(col<0 || col>=Horizon_SCAN){
                continue;
            }

            float range = sqrt(point.x*point.x+point.y+point.y+point.z*point.z);
            if(range<0.2) continue;
//            rangeMat.at<Eigen::Vector4d>(row,col)[0]=point.x;
//            rangeMat.at<Eigen::Vector4d>(row,col)[1]=point.y;
//            rangeMat.at<Eigen::Vector4d>(row,col)[2]=point.z;
//            rangeMat.at<Eigen::Vector4d>(row,col)[3]=range;
            rangeMat.at<float>(row,col)=range;

            int index = col + row *Horizon_SCAN;


            Point_col[index]=col;
            Point_row[index]=row;
            Point_range[index]=range;


            valid_points_num++;
            point.intensity = (float)row+(float )col/10000.0;
            fullCloud->points[index]=point;


        }

    }


    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffx, diffy,diffz,angle;
        for(int j=0;j<Horizon_SCAN;j++){
            for(int i=0;i<groundScanInd;i++){
                lowerInd = j + (i)*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;
                if(fullCloud->points[lowerInd].intensity==-1 || fullCloud->points[upperInd].intensity==-1){
                    groundMat.at<int8_t>(i,j)=-1;
                    continue;
                }
//                diffx = rangeMat.at<Eigen::Vector4d >(i,j)[0]-rangeMat.at<Eigen::Vector4d >(i,j)[0];
//                diffy = rangeMat.at<Eigen::Vector4d >(i,j)[1]-rangeMat.at<Eigen::Vector4d >(i,j)[1];
//                diffz = rangeMat.at<Eigen::Vector4d >(i,j)[2]-rangeMat.at<Eigen::Vector4d >(i,j)[2];

                diffx = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffy = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffz = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;


                angle = atan2(diffz, sqrt(diffx*diffx + diffy*diffy) ) * 180 / M_PI;

                if(abs(angle-0.0)<=10){
                    groundMat.at<int8_t>(i,j)=1;
                    groundMat.at<int8_t>(i+1,j)=1;


                }


            }
        }

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<Eigen::Vector4d>(i,j)[0] == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }

        if(pubGroundCloud.getNumSubscribers()!=0){
            //ROS_INFO("1");
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }

    }


    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)//-1代表地面和滤除的无效点
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {//接下来对每一行进行分类

            startRingIndex[i] = sizeOfSegCloud-1 + 5;//？

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){//有标签的和地面点
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){//无效的分类点
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    //segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }

        if(pubSegmentedCloud.getNumSubscribers()!=0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        //segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }

    }



    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        std::vector<bool> lineCountFlag(N_SCAN,false);

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
//                d1 = std::max(rangeMat.at<Eigen::Vector4d>(fromIndX, fromIndY)[3],
//                              rangeMat.at<Eigen::Vector4d>(thisIndX, thisIndY)[3]);
//                d2 = std::min(rangeMat.at<Eigen::Vector4d>(fromIndX, fromIndY)[3],
//                              rangeMat.at<Eigen::Vector4d>(thisIndX, thisIndY)[3]);

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                //角度越大越接近
                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){//横跨3条线就可以？
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points，outliers 标记为9999
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void publish(){
        sensor_msgs::PointCloud2 laserCloudTemp;

        if(pubOutlierCloud.getNumSubscribers()!=0){
            pcl::toROSMsg(*outlierCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloud_header.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubOutlierCloud.publish(laserCloudTemp);
        }

        // segmented cloud with ground
        if(pubSegmentedCloud.getNumSubscribers()!=0){
            pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloud_header.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubSegmentedCloud.publish(laserCloudTemp);
        }

        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloud_header.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloud_header.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubGroundCloud.publish(laserCloudTemp);
        }



    }
};

int main(int argc, char **argv){
    ros::init(argc,argv,"adaCt_odometry");
    ROS_INFO("PREPROCESS START!");
    Preprocess lp;
    ros::spin();

    return 0;
}