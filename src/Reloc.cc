#include <Reloc.h>

#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Frame.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"
#include <string>
#include <System.h>

#include <iostream>

#include <mutex>

using namespace std;

namespace ORB_SLAM2
{
    Reloc::Reloc(ORB_SLAM2::System *pslam) : mpORBVocabulary_R(pslam->mpVocabulary), mpSLAM(pslam)
    {
        cout << "define Reloc" << endl;
    }
    //读取图片并显示
    bool Reloc::LoadImage(string s)
    {
        cout << "Loading Image to Relocalization" << endl;
        R_Image = cv::imread(s, CV_LOAD_IMAGE_UNCHANGED);
        if (R_Image.empty())
        {
            cout << "Please input right image path" << endl;
            return false;
        }
        string sWindowName = "Reloc_Image_" + to_string(R_ImgNum) + "_" + s.erase(0, s.length() - 10);
        cv::namedWindow(sWindowName, cv::WINDOW_AUTOSIZE);
        cv::imshow(sWindowName, R_Image);
        cout << "Load Image Finished!" << endl;
        return true;
    }
    //处理需要重定位的图片（构造Frame）
    bool Reloc::ProcessImage()
    {
        cout << "Processing Image to Relocalization" << endl;
        mRelocFrame = Frame(R_Image, 999, mpSLAM->mpTracker->mpORBextractorLeft, mpSLAM->mpTracker->mpORBVocabulary, mpSLAM->mpTracker->mK, mpSLAM->mpTracker->mDistCoef, mpSLAM->mpTracker->mbf, mpSLAM->mpTracker->mThDepth);
        cout << "Processing Image Finsished!" << endl;
    }
    //重定位函数
    bool Reloc::Relocalization()
    {
        R_ImgNum++;
        cout << "Begin Relocation" << endl;

        // Compute Bag of Words Vector
        mRelocFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        // cout <<"Tracking::Relocalization() : checkpoint1" << endl;
        vector<KeyFrame *> vpCandidateKFs = mpSLAM->mpTracker->mpKeyFrameDB->DetectRelocalizationCandidates(&mRelocFrame);

        if (vpCandidateKFs.empty())
        {
            cout << "没有找到候选关键帧" << endl;
            return false;
        }
        const int nKFs = vpCandidateKFs.size();
        cout << "最后找到的候选帧数量为：" << nKFs << endl;

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        // cout <<"Tracking::Relocalization() : checkpoint2, nKFs = " << nKFs << endl;
        ORBmatcher matcher(0.75, true);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        cout << "Try to localize in " << nKFs << " KeyFrames." << endl;
        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF, mRelocFrame, vvpMapPointMatches[i]);
                cout << "通过BoW匹配得到的匹配地图点数为：" << nmatches << endl;
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver *pSolver = new PnPsolver(mRelocFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                    // cout <<"Tracking::Relocalization() : checkpoint2.1, nCandidates = " << nCandidates << " added to the pSolver." << endl;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        // cout <<"Tracking::Relocalization() : checkpoint3" << endl;
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty())
                {
                    Tcw.copyTo(mRelocFrame.mTcw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mRelocFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mRelocFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mRelocFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < mRelocFrame.N; io++)
                        if (mRelocFrame.mvbOutlier[io])
                            mRelocFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // // If few inliers, search by projection in a coarse window and optimize again
                    // if (nGood < 50)
                    // {
                    //     int nadditional = matcher2.SearchByProjection(mRelocFrame, vpCandidateKFs[i], sFound, 10, 100);

                    //     if (nadditional + nGood >= 50)
                    //     {
                    //         nGood = Optimizer::PoseOptimization(&mRelocFrame);

                    //         // If many inliers but still not enough, search by projection again in a narrower window
                    //         // the camera has been already optimized with many points
                    //         if (nGood > 30 && nGood < 50)
                    //         {
                    //             sFound.clear();
                    //             for (int ip = 0; ip < mRelocFrame.N; ip++)
                    //                 if (mRelocFrame.mvpMapPoints[ip])
                    //                     sFound.insert(mRelocFrame.mvpMapPoints[ip]);
                    //             nadditional = matcher2.SearchByProjection(mRelocFrame, vpCandidateKFs[i], sFound, 3, 64);

                    //             // Final optimization
                    //             if (nGood + nadditional >= 50)
                    //             {
                    //                 nGood = Optimizer::PoseOptimization(&mRelocFrame);

                    //                 for (int io = 0; io < mRelocFrame.N; io++)
                    //                     if (mRelocFrame.mvbOutlier[io])
                    //                         mRelocFrame.mvpMapPoints[io] = NULL;
                    //             }
                    //         }
                    //     }
                    // }

                    // // If the pose is supported by enough inliers stop ransacs and continue
                    // if (nGood >= 50)
                    // {
                    //     bMatch = true;
                    //     break;
                    // }
                    bMatch = true;
                    break;
                }
            }
        }
        // // cout <<"Tracking::Relocalization() : checkpoint4 Solver finished, current pose mCurrentFrame.mTcw = " << mCurrentFrame.mTcw << endl;

        if (!bMatch)
        {
            // cout <<"Tracking::Relocalization() : checkpoint5 !bMatch" << endl;
            return false;
        }
        else
        {
            ///将重定位的位姿送到绘图类
            mpSLAM->mpMapDrawer->mRelocPose = mRelocFrame.mTcw.clone();
            cout << mRelocFrame.mTcw << endl;
            // cout <<"Tracking::Relocalization() : checkpoint5 true, mnLastRelocFrameId = " << mnLastRelocFrameId << endl;
            // mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }
        // return true;
    }
} //namespace ORB_SLAM
