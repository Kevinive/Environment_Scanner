/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "env_rebuilder/point_cloud.h"


namespace env_rebuilder
{

PointCloud::PointCloud(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double unit_scale)
: x_offset_(-min_x), y_offset_(-min_y), z_offset_(-min_z),
  x_size_(max_x-min_x), y_size_(max_y-min_y), z_size_(max_z-min_z), unit_scale_(unit_scale),
  point_map_(max_x-min_x, max_y-min_y, max_z-min_z, unit_scale)
{}


void PointCloud::insertLidarPoint ( LidarPoint::Ptr lidar_point )
{
    // 确定ID不会重复
    std::vector<LidarPoint::Ptr> result;
    bool haveSame = false;
    point_map_.get(lidar_point->pos_.x() + x_offset_, lidar_point->pos_.y() + y_offset_, lidar_point->pos_.z() + z_offset_, result);
    for (LidarPoint::Ptr lp : result){
        if(lp->id_ == lidar_point->id_){
            haveSame = true;
            break;
        }
    }
    if ( !haveSame )
    {
        point_map_.save(lidar_point, lidar_point->pos_.x() + x_offset_, lidar_point->pos_.y() + y_offset_, lidar_point->pos_.z() + z_offset_);
    }

}

void PointCloud::insertLidarPoint ( std::vector<LidarPoint::Ptr>& lpvec ){
    for(LidarPoint::Ptr lp : lpvec){
        insertLidarPoint(lp);
    }
}

void PointCloud::getNearestPoint( double x, double y, double z , std::vector<LidarPoint::Ptr>& result){
    point_map_.get(x + x_offset_, y + y_offset_, z + z_offset_, result);
}

void PointCloud::getNeighborPoint( double x, double y, double z , int step, std::vector<LidarPoint::Ptr>& result){
    point_map_.getNeighbor(x + x_offset_, y + y_offset_, z + z_offset_, step, result);
}

void PointCloud::getAllPoint(std::vector<LidarPoint::Ptr>& result){
    point_map_.getAllObject(result);
}

PointCloud::Ptr PointCloud::createPointCloud(
        double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double unit_scale)
{
    return PointCloud::Ptr(
            new PointCloud( min_x, max_x, min_y, max_y, min_z, max_z, unit_scale)
    );
}

PointCloud::Ptr PointCloud::createPointCloud(
        std::vector<LidarPoint::Ptr>& lpvec, double unit_scale)
{
    double max_x, max_y, max_z, min_x, min_y, min_z;
    max_x = max_y = max_z = -100.0;
    min_x = min_y = min_z = 100.0;
    for(LidarPoint::Ptr lp : lpvec){
        if(lp->pos_.x() > max_x){
            max_x = lp->pos_.x();
        }else if(lp->pos_.x() < min_x){
            min_x = lp->pos_.x();
        }

        if(lp->pos_.y() > max_y){
            max_y = lp->pos_.y();
        }else if(lp->pos_.y() < min_y){
            min_y = lp->pos_.y();
        }

        if(lp->pos_.z() > max_z){
            max_z = lp->pos_.z();
        }else if(lp->pos_.z() < min_z){
            min_z = lp->pos_.z();
        }
    }
    PointCloud::Ptr pcp =  createPointCloud(min_x, max_x, min_y, max_y, min_z, max_z, unit_scale);
    pcp->insertLidarPoint(lpvec);
    return pcp;
}
}
