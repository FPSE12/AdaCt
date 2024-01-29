//
// Created by wjj on 2023/11/15.


#include "AdaCt/utility.h"
#include <opencv/cv.h>
//clion need use the node to build generate_message first
//https://www.cnblogs.com/AdamTang/p/15567792.html
#include "AdaCt/cloud_info.h"

template <typename PointT>
void MYremoveNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out,
                                std::vector<int> &index)
{
    // If the clouds are not the same, prepare the output
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize (cloud_in.points.size ());
    }
    // Reserve enough space for the indices
    index.resize (cloud_in.points.size ());

    // If the data is dense, we don't need to check for NaN
    if (cloud_in.is_dense)
    {
        // Simply copy the data
        cloud_out = cloud_in;
        for (std::size_t j = 0; j < cloud_out.points.size (); ++j)
            index[j] = static_cast<int>(j);
    }
    else
    {
        std::size_t j = 0;
        for (std::size_t i = 0; i < cloud_in.points.size (); ++i)
        {
            if (!std::isfinite (cloud_in.points[i].x) ||
                !std::isfinite (cloud_in.points[i].y) ||
                !std::isfinite (cloud_in.points[i].z))
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            index[j] = static_cast<int>(i);
            j++;
        }
        if (j != cloud_in.points.size ())
        {
            // Resize to the correct size
            cloud_out.points.resize (j);
            index.resize (j);
        }

        cloud_out.height = 1;
        cloud_out.width  = static_cast<std::uint32_t>(j);

        // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
        cloud_out.is_dense = true;
    }
}

// template <typename PointT>
// void RemoveOutLiers(const pcl::PointCloud<PointT>::Ptr & cloud_in,
//                     pcl::PointCloud<PointT>::Ptr cloud_out){
//                         pcl::StatisticalOutlierRemoval<PointT> sor;
//                         sor.setInputCloud(cloud_in);
//                         sor.setMeanK(50);
//                         sor.setStddevMulThresh(1.0);
//                         sor.filter(cloud_out);
//                     }



class Preprocess: public configParam{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;

    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudInRing;

    pcl::StatisticalOutlierRemoval<PointXYZIRT> sor;
    pcl::StatisticalOutlierRemoval<PointXYZIRT> r_sor;

    pcl::PointCloud<PointXYZIRT>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointXYZIRT>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointXYZIRT>::Ptr groundCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr segmentedCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointXYZIRT>::Ptr outlierCloud;

    std::deque<sensor_msgs::PointCloud2> lidarStore_queue;

    PointXYZIRT nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    //cloud_msgs::cloud_info segMsg; // info of segmented cloud
    AdaCt::cloud_info segMsg;
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    std::vector<int> startRingIndex;
    std::vector<int> endRingIndex;

    std::vector<bool> segmentedCloudGroundFlag;
    std::vector<uint32_t >segmentedCloudColInd;
    std::vector<float> segmentedCloudRange;

    std::mutex mtx;

public:
    Preprocess()
    {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &Preprocess::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<AdaCt::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIRT>());

        fullCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        fullInfoCloud.reset(new pcl::PointCloud<PointXYZIRT>());

        groundCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        segmentedCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointXYZIRT>());
        outlierCloud.reset(new pcl::PointCloud<PointXYZIRT>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);


        startRingIndex.assign(N_SCAN, 0);
        endRingIndex.assign(N_SCAN, 0);

        segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];


        sor.setMeanK(20);
        sor.setStddevMulThresh(1.0);
        r_sor.setMeanK(20);
        r_sor.setStddevMulThresh(1.0);

    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~Preprocess(){}

    void copyPointCloud(const sensor_msgs::PointCloud2 laserCloudMsg){

        cloudHeader = laserCloudMsg.header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        //pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        MYremoveNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
//        sor.setInputCloud(laserCloudIn);
//        sor.setMeanK(20);
//        sor.setStddevMulThresh(1.0);
//        sor.filter(*laserCloudIn);
        // RemoveOutLiers(*laserCloudIn,*laserCloudIn);
        // have "ring" channel in the cloud6


        }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        std::lock_guard<std::mutex> lock(mtx);
        lidarStore_queue.push_back(*laserCloudMsg);



        //feature

    }

    void runpreprocess(){
        if(lidarStore_queue.empty()){
            return;
        }
        auto start = std::chrono::steady_clock::now();

        // 1. Convert ros message to pcl point cloud
        copyPointCloud(lidarStore_queue.front());
        lidarStore_queue.pop_front();
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();

        auto end=std::chrono::steady_clock::now();
        //about 20ms
        ROS_INFO("INIT COST: %f ms",
                 std::chrono::duration<double, std::milli>(end - start).count());
    }

    void findStartEndAngle(){
        // start and end orientation of this cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                         laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
        ROS_INFO("START :%f",segMsg.startOrientation);
        ROS_INFO("END: %f",segMsg.endOrientation);
    }

    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointXYZIRT thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.timestamp = laserCloudIn->points[i].timestamp;
            // find the row and column index in the iamge for this point

            rowIdn = laserCloudIn->points[i].ring;

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;


//            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
//            if(horizonAngle<0) horizonAngle += 360;
//            columnIdn = round(horizonAngle/ang_res_x);
//            columnIdn = ((int)(columnIdn + 0.75 * Horizon_SCAN)) % Horizon_SCAN;
//            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
//                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;


            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < 0.2)
                continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }


    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        // 一定的线数下寻找地面
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle) <= 5){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0){
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
            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

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
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segmentedCloudColInd[sizeOfSegCloud] = j;
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            endRingIndex[i] = sizeOfSegCloud-1 - 5;
            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }

        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[32] = {false};

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


    void publishCloud(){
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "map";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "map";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "adact_odometry");
    ROS_INFO("PREPROCESS START!");

    Preprocess IP;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        IP.runpreprocess();

        rate.sleep();
    }


    ros::spin();
    return 0;
}
