/*
 *  Copyright (C) 1997-2015 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *  Authors :
 *       Victor Arribas Raigadas <v.arribas.urjc@gmail.com>
 */


#include "pcl_qtwidget.hpp"

QPCLViewer::QPCLViewer(QWidget* parent, Qt::WFlags f):
    QVTKWidget(parent, f),
    qvtkWidget(this),
    pclviewer(new pcl::visualization::PCLVisualizer("PCLVisualizer", false))
{
    qvtkWidget->SetRenderWindow(pclviewer->getRenderWindow());
    pclviewer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
    pclviewer->initCameraParameters ();

    initUi();
}

QPCLViewer::~QPCLViewer()
{
    this->hide();
    delete pclviewer;
}


void
QPCLViewer::initUi()
{
    this->resize(720, 500);
    pclviewer->setBackgroundColor(0, 0, 0);
}


void
QPCLViewer::pushCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pclviewer->removeAllPointClouds();
    pclviewer->addPointCloud<pcl::PointXYZ> (cloud);
    pclviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    pclviewer->addCoordinateSystem (1.0);
}

void
QPCLViewer::pushCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pclviewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclviewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
    pclviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    pclviewer->addCoordinateSystem (1.0);
}
