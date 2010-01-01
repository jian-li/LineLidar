# LineLidar
Line lidar used to get the pointcloud!
This package contains the 1st generation neato lidar driver! 

- NeotoLidarDriver 
 
 This is the neoto lidar driver node, which is changed from a xv11 lidar hack project.
 
 - 2dSLAM
 
 This is the 2d slam node, currently icp algorithm was tested! In the future the gmapping, hector slam, karto slam will be tested.
 
 - 2d path planning
 
 # Hector SLAM算法核心部分分析
 ```
   /*
    *
   */
   Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, ConcreteOccGridMapUtil& gridMapUtil, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix, int maxIterations)
    {
        if (drawInterface)
        {
            drawInterface->setScale(0.05f);
            drawInterface->setColor(0.0f,1.0f, 0.0f);
            drawInterface->drawArrow(beginEstimateWorld);

            Eigen::Vector3f beginEstimateMap(gridMapUtil.getMapCoordsPose(beginEstimateWorld));

            drawScan(beginEstimateMap, gridMapUtil, dataContainer);

            drawInterface->setColor(1.0,0.0,0.0);
        }

        if (dataContainer.getSize() != 0)
        {

            Eigen::Vector3f beginEstimateMap(gridMapUtil.getMapCoordsPose(beginEstimateWorld));

            Eigen::Vector3f estimate(beginEstimateMap);

            estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);


            int numIter = maxIterations;


            for (int i = 0; i < numIter; ++i) {
                //std::cout << "\nest:\n" << estimate;

                estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
                //notConverged = estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

                if(drawInterface)
                {
                    float invNumIterf = 1.0f/static_cast<float> (numIter);
                    drawInterface->setColor(static_cast<float>(i)*invNumIterf,0.0f, 0.0f);
                    drawInterface->drawArrow(gridMapUtil.getWorldCoordsPose(estimate));
                    //drawInterface->drawArrow(Eigen::Vector3f(0.0f, static_cast<float>(i)*0.05, 0.0f));
                }

                if(debugInterface){
                    debugInterface->addHessianMatrix(H);
                }
            }

            if (drawInterface)
            {
                drawInterface->setColor(0.0,0.0,1.0);
                drawScan(estimate, gridMapUtil, dataContainer);
            }


            estimate[2] = util::normalize_angle(estimate[2]);

            covMatrix = Eigen::Matrix3f::Zero();
            //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0).inverse());

            covMatrix = H;

            return gridMapUtil.getWorldCoordsPose(estimate);
        }

        return beginEstimateWorld;
    }
 ```
# camera caliratoin 
To use the camera calibration function, you can use the code in 2dslam/tools, you should first setting the calibration file, include the inner corner numbers, mm per unit, fixratio, web cam device number, image numbers.
 
Further advice can be found here:
http://answers.opencv.org/question/7554/results-of-camera-calibration-vary/
=======
