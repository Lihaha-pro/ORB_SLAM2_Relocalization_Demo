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
#include <unistd.h>
#include "Viewer.h"
#include <Reloc.h>
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath) : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
                                                                                                                                           mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        //计算每一帧所持续的时间
        mT = 1e3 / fps;
        //图片长宽参数
        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        //设置默认值
        if (mImageWidth < 1 || mImageHeight < 1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }
        //读取视角
        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    void Viewer::Run()
    {
        mbFinished = false;
        mbStopped = false;
        mbUseIMU = false;

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 2048, 1536); ///pangolin窗口大小

        // 3D Mouse handler requires depth testing to be enabled
        // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        //在OpenGL中使用颜色混合
        glEnable(GL_BLEND);
        //选择混合模式
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        // 定义显示面板大小，orbslam中有左右两个面板，左边显示一些按钮，右边显示图形
        // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
        // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
        // 最后一个参数（-1024.0f/768.0f）为显示长宽比
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                    .SetHandler(new pangolin::Handler3D(s_cam));
        //创建一个欧式变换矩阵，存储当前的相机位姿
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
        ///创建一个欧式变换矩阵，存储当前的相机位姿
        pangolin::OpenGlMatrix Twc_R;
        Twc_R.SetIdentity();

        cv::namedWindow("ORB-SLAM2: Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = false;

        int img_num = 0;
        //更新绘制的内容
        while (1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            mpMapDrawer->GetRelocOpenGLCameraMatrix(Twc_R); ///获取重定位位姿到Twc_R
            if (menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if (menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }
            if (menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if (!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            // step 3：绘制地图和图像(3D部分)
            // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            mpMapDrawer->DrawRelocFrame(Twc_R); ///根据Twc_R进行绘制
            if (menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            cv::Mat im = mpFrameDrawer->DrawFrame();
            cv::imshow("ORB-SLAM2: Current Frame", im);
            std::string homepath = getenv("HOME");
            if (mbSaveImg && mpFrameDrawer->mbSaveImage)
            {
                char ad[128] = {0};
                if (mbUseIMU)
                {
                    sprintf(ad, "%s/Pictures/ORB-SLAM2-IMU/ORB-SLAM2_img_%06d.jpg", homepath.c_str(), ++img_num);
                }
                else
                {
                    sprintf(ad, "%s/Pictures/ORB-SLAM2/ORB-SLAM2_img_%06d.jpg", homepath.c_str(), ++img_num);
                }

                cv::imwrite(ad, im);
                // Please see System.cc, I need to add current path.
                mpFrameDrawer->mbSaveImage = false;
            }
            cv::waitKey(mT);

            if (menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if (Stop())
            {
                while (isStopped())
                {
                    usleep(3000);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}
