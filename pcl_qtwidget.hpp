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


#pragma once

//Qt
#include <QWidget>
#include <QVTKWidget.h>

//VTK
#include <vtkRenderWindow.h>

//PCL
#include <pcl/visualization/pcl_visualizer.h>


class QPCLViewer : public QVTKWidget
{
    //Q_OBJECT

public:
    QPCLViewer(QWidget* parent = NULL, Qt::WFlags f = 0);
    ~QPCLViewer();

    //// render cloud PointCloud. Also remove any previous cloud to avoid id collision.
    void pushCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    void pushCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

    //// use explicit variable for VTK stuff (but qvtkWidget==this).
    QVTKWidget* const qvtkWidget;

    //// allow external access to pcl visualizer to hack it
    pcl::visualization::PCLVisualizer* const pclviewer;

protected:
    void initUi();

};
