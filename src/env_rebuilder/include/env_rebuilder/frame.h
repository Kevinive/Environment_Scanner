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

#ifndef FRAME_H
#define FRAME_H

#include "env_rebuilder/common_include.h"
#include "env_rebuilder/feature.h"
#include "env_rebuilder/lidarpoint.h"
#include "env_rebuilder/camera.h"

namespace env_rebuilder
{

class Feature;
class LidarPoint;
class Camera;

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                   id_;            // id of this frame
    double                          time_stamp_;    // when it is recorded
    SE3                             T_c_w_;         // transform from world to camera
    Camera::Ptr                     camera_;        // Pinhole RGBD Camera model
    Mat                             color_;         // color image
    string                          img_add_;       // color image address
    LidarPoint::Ptr                 c_point_;        // data of central distance
    std::vector<Feature::Ptr>       features_;      // key points in image
    bool                            is_key_frame_;  // whether a key-frame

private:
    static unsigned long            factory_id_;    // factory id

public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), string ad=string(), LidarPoint::Ptr lp=nullptr);
    ~Frame();
    
    static Frame::Ptr createFrame();

    void addFeature( const Feature::Ptr feature);

};

}

#endif // FRAME_H
