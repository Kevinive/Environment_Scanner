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

#include "env_rebuilder/lidarpoint.h"
#include "env_rebuilder/frame.h"


namespace env_rebuilder
{

LidarPoint::LidarPoint()
: id_(-1), pos_(Vector3d(0,0,0)), color_(Vector3d(0,0,0)), isAccurate(false)
{

}

LidarPoint::LidarPoint ( long unsigned int id, const Vector3d& position, const Vector3d& color, bool vali)
: id_(id), pos_(position), color_(color), isAccurate(vali)
{

}

void LidarPoint::insertObservedFrame ( Frame* frame )
{
    if ( observed_frame_.find(frame->id_) == observed_frame_.end() )
    {
        observed_frame_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        observed_frame_[ frame->id_ ] = frame;
    }
}


PointT LidarPoint::getPCLPoint(){
    PointT p(color_(0,0), color_(1,0), color_(2,0));
    p.x = pos_(0,0);
    p.y = pos_(1,0);
    p.z = pos_(2,0);
    return p;
}

LidarPoint::Ptr LidarPoint::createLidarPoint()
{
    return LidarPoint::Ptr(
        new LidarPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) , false)
    );
}

LidarPoint::Ptr LidarPoint::createLidarPoint (
    const Vector3d& pos_world, 
    const Vector3d& color,
    bool isAccurate
    )
{
    return LidarPoint::Ptr(
        new LidarPoint( factory_id_++, pos_world, color, isAccurate )
    );
}

LidarPoint::Ptr LidarPoint::createLidarPoint (
    const Eigen::Vector4d& pos_world,
    const Vector3d& color,
    bool isAccurate
)
{
    return createLidarPoint(Vector3d(pos_world[0], pos_world[1], pos_world[2]), color, isAccurate);
}

LidarPoint::Ptr LidarPoint::createLidarPoint (
        const Eigen::Vector4d& pos_world,
        bool isAccurate
)
{
    return createLidarPoint(Vector3d(pos_world[0], pos_world[1], pos_world[2]), Vector3d(255, 255, 255), isAccurate);
}

bool LidarPoint::addObservedFrame(Frame* frame){

    Vector3d p_cam = frame->camera_->world2camera(pos_, frame->T_c_w_);
    if(p_cam.x() < 0.1) return false;
    Vector2d p_pixel = frame->camera_->camera2pixel(p_cam);
    if(p_pixel.x() > 0.0 && p_pixel.x() < frame->camera_->width_ && p_pixel.y() > 0.0 && p_pixel.y() < frame->camera_->hight_){
        insertObservedFrame(frame);
        return true;
    }
    return false;
}

bool LidarPoint::isValid(){
    return isAccurate;
}

unsigned long LidarPoint::factory_id_ = 0;

}
