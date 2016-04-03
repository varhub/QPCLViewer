/*
 *  Copyright (C) 2015 Victor Arribas <v.arribas.urjc@gmail.com>
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


#include <QApplication>
#include <QMainWindow>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>

#include "pcl_qtwidget.hpp"



int main (int argc, char *argv[])
{
    QApplication app (argc, argv);


    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Genarating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
    }


    // A) Simple usage
    // Use widget 'AS IS'
    QPCLViewer widget;
    widget.setWindowTitle("QPCLVisualizer");
    widget.show ();

    widget.pushCloud(basic_cloud_ptr);
    widget.pushCloud(point_cloud_ptr);

    // B) As Window component
    // Embed it into a MainWindow, with custom SIZE and POSITION
    QMainWindow win;
    win.setWindowTitle("MainWindow");
    win.resize(500,500);
    QPCLViewer viewer(&win);
    viewer.setGeometry(50, 50, 400,400);
    win.show();

    viewer.pushCloud(point_cloud_ptr);

    // C) complex composition
    // Embedded an with multiple viewports
    QMainWindow win2;
    win2.setWindowTitle("MainWindow2");
    win2.resize(800,400);
    QPCLViewer viewer1(&win2), viewer2(&win2);
    viewer1.setGeometry(0, 0, 400,400);
    viewer2.setGeometry(400, 0, 400,400);
    win2.show();

    int v1(0), v2(0);
    viewer1.pclviewer->createViewPort(0.0, 0.0, 0.5, 0.5, v1);
    viewer1.pclviewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer1.pclviewer->setBackgroundColor(1,1,1, v2);

    viewer1.pclviewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "cloud", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer1.pclviewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "rgbcloud", v2);

    viewer2.pclviewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "rgbcloud");

    return app.exec ();
}
