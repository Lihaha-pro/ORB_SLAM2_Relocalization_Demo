/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

    MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : mpMap(pMap)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    }

    void MapDrawer::DrawMapPoints()
    {
        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
        {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
    {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        if (bDrawKF)
        {
            for (size_t i = 0; i < vpKFs.size(); i++)
            {
                KeyFrame *pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }

        if (bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            for (size_t i = 0; i < vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty())
                {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
                    {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
                {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize; //相机大小
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }
    //绘制重定位的帧
    void MapDrawer::DrawRelocFrame(pangolin::OpenGlMatrix &Twc)
    {
        //cout << "Call DrawRelocFrame() function" << endl;
        //相机模型大小：宽度占总宽度比例为0.08
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        //百度搜索：glPushMatrix 百度百科
        glPushMatrix();

        //将4*4的矩阵Twc.m右乘一个当前矩阵
        //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
        //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
        //一个是整型,一个是浮点数类型
#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        //设置绘制图形时线的宽度
        glLineWidth(mCameraLineWidth);
        //!设置当前颜色为绿色(相机图标显示为绿色)
        glColor3f(1.0f, 0.0f, 0.0f);
        //用线将下面的顶点两两相连
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }

    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if (!mCameraPose.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        }
        else
            M.SetIdentity();
    }
    ///自定义获得重定位位姿的openGl格式姿态
    //输入的是用于保存pangolin格式的mat，程序内部根据mRelocPose是否有值，从而为这个姿态赋值
    void MapDrawer::GetRelocOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        // cout << mRelocPose << endl;//看一下当前位姿是什么
        ///自己随便写写
        // mRelocPose.at<float>(0, 3) = 1;
        // mRelocPose.at<float>(1, 3) = 2;
        if (!mRelocPose.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mRelocPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mRelocPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        }
        else
            M.SetIdentity();
    }

} //namespace ORB_SLAM
