
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left, bottom+50), cv::FONT_ITALIC, 0.5, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left, bottom+125), cv::FONT_ITALIC, 0.5, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::KeyPoint> kptsPrev_inlier;
    std::vector<cv::KeyPoint> kptsCurr_inlier;
    std::vector<cv::DMatch> match_inlier;
    double dist=0;
    double cnt=0;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        cv::KeyPoint kpCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpPrev = kptsPrev.at(it1->queryIdx);
        if(boundingBox.roi.contains(kpCurr.pt) )
        {
            kptsCurr_inlier.push_back(kpCurr);
            kptsPrev_inlier.push_back(kpPrev);
            match_inlier.push_back(*it1);
            //cout<<cv::norm(kpCurr.pt - kpPrev.pt)<<"  "<<endl;
            dist=dist+cv::norm(kpCurr.pt - kpPrev.pt);
            cnt++;
        }

    }
    double mean_dist=dist/cnt;
    double thres_ratio = 1.5;
    std::vector<cv::DMatch> match_inlier_temp;
    for (int i= 0; i < kptsCurr_inlier.size();i++)
    {
        // Euclidean distance between previous and current key points
        double dist = cv::norm(kptsCurr_inlier[i].pt - kptsPrev_inlier[i].pt);

        // Remove keypoint matches when the Euclidean distance
        // farther than the mean keypoint distance
        if (dist <= mean_dist * thres_ratio)
        {
            match_inlier_temp.push_back(match_inlier[i]);
        } 
     }
     //cout<<match_inlier_temp.size()<<" "<<match_inlier.size()<<endl;
     boundingBox.kptMatches=match_inlier_temp;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
      // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1.0 / frameRate;
    TTC = -dT / (1 - medDistRatio);
 
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1/frameRate;        // time between two measurements in seconds
   
    
    // find closest distance to Lidar points within ego lane
    std::vector<double> lidarPointsPrev_x;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    { 
        lidarPointsPrev_x.push_back(it->x);
    }
    sort(lidarPointsPrev_x.begin(),lidarPointsPrev_x.end());

    std::vector<double> lidarPointsCurr_x;
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        lidarPointsCurr_x.push_back(it->x);
    }
    sort(lidarPointsCurr_x.begin(),lidarPointsCurr_x.end());

    //double minXPrev=lidarPointsPrev_x[0];
    //double minXCurr=lidarPointsCurr_x[0];
    double minXPrev=lidarPointsPrev_x[lidarPointsPrev_x.size()/2];
    double minXCurr=lidarPointsCurr_x[lidarPointsCurr_x.size()/2];
    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
    //cout<<minXCurr<<endl;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //std::map<int, int> multimap={};
    vector<pair<int,int>> tm;
    vector<pair<pair<int,int>,int>> multimap;
    vector<pair<pair<int,int>,int>> multimap_temp;
    //vector<pair<pair<int,int>,int>> multimap;
    for (auto it1 = matches.begin(); it1 != matches.end(); it1++)
    {   
        vector<int> index_cur;
        vector<int> index_pre;
        
        //for (auto it2 = currFrame.boundingBoxes.begin(); it2 != currFrame.boundingBoxes.end(); it2++)
        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (auto it2 = currFrame.boundingBoxes.begin(); it2 != currFrame.boundingBoxes.end(); ++it2)
        {
            if ((*it2).roi.contains(currFrame.keypoints[(*it1).trainIdx].pt))
            {
                index_cur.push_back((*it2).boxID);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
       
        
        for (vector<BoundingBox>::iterator it3 = prevFrame.boundingBoxes.begin(); it3 != prevFrame.boundingBoxes.end(); ++it3)
        {
            if ((*it3).roi.contains(prevFrame.keypoints[(*it1).queryIdx].pt))
            {
                index_pre.push_back((*it3).boxID);
            }

        } // eof loop over all bounding boxes

        for(auto cur:index_cur)
        {
            for(auto pre:index_pre)
            {tm.push_back({cur,pre});}

        }
        
        
    } // eof loop over all Lidar points


    //cout<<tm.size()<<endl;
    //cout<<currFrame.boundingBoxes.size()<<endl;
    vector<pair<int,int>> ps;
    for (auto it4 = currFrame.boundingBoxes.begin(); it4 != currFrame.boundingBoxes.end(); ++it4)
    {
        for (auto it5 = prevFrame.boundingBoxes.begin(); it5 != prevFrame.boundingBoxes.end(); ++it5)
        {
            pair<int,int> p={it4->boxID,it5->boxID};
            ps.push_back(p);
            multimap.push_back({p,0});
        }
    }
    
    for(auto m:multimap)
    {
        int cnt=0;
        for(auto n:tm)
        {
            if(m.first==n)
            {
                cnt++;
            }
        }
        m.second=cnt;
        multimap_temp.push_back(m);
        //cout<<m.first.first<<()" "<<m.first.second<<" "<<m.second<<endl;
    }
    pair<pair<int,int>,int> pmax={{-1,-1},-1};
    vector<pair<pair<int,int>,int>> pmaxs;
    for (auto it6 = currFrame.boundingBoxes.begin(); it6 != currFrame.boundingBoxes.end(); ++it6)
    {
        for(auto n:multimap_temp)
        {
            
            if(n.first.first==it6->boxID)
            {
                //cout<<n.first.first<<" "<<it6->boxID<<endl;
                if(n.second>pmax.second)
                {
                    pmax=n;
                    
                }
            }
        }
        pmaxs.push_back(pmax);
        pmax={{-1,-1},-1};
        //cout<<it6->boxID<<" "<<pmax.first.first<<" "<<pmax.first.second<<" "<<pmax.second<<endl;
        
    }
    pair<pair<int,int>,int> p2r_max={{-1,-1},-1};  
    vector<pair<pair<int,int>,int>> p2r_maxs;
    for (auto it7 = prevFrame.boundingBoxes.begin(); it7 != prevFrame.boundingBoxes.end(); ++it7)
    {
        for(auto mm:pmaxs)
        {
            if(it7->boxID==mm.first.second)
            {
                if (mm.second>p2r_max.second)
                {
                    p2r_max=mm;
                }
                
            }
        }
        if(p2r_max.second!=-1)
        {
            p2r_maxs.push_back(p2r_max);
        }
        p2r_max={{-1,-1},-1};
    }
    for(auto mm:p2r_maxs)
    {
        bbBestMatches.insert({mm.first.second,mm.first.first});
        //cout<<mm.first.first<<" "<<mm.first.second<<" "<<mm.second<<endl;
    }
}
