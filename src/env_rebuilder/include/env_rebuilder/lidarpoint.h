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

#ifndef LIDARPOINT_H
#define LIDARPOINT_H

#include "env_rebuilder/common_include.h"


namespace env_rebuilder
{

class Frame;

class LidarPoint
{
public:
    typedef std::shared_ptr<LidarPoint> Ptr;
    unsigned long      id_;        // ID
    Vector3d    pos_;       // Position in world
    Vector3d    color_;     // Color of the Point

    bool isAccurate;

    std::unordered_map<unsigned long, Frame* >  observed_frame_;        // frames contained lidarpoint

private:
    static unsigned long factory_id_;    // factory id

public:
    LidarPoint();
    LidarPoint(
        unsigned long id, 
        const Vector3d& position,
        const Vector3d& color,
        bool isAccurate=false
    );

    void insertObservedFrame ( Frame* frame );

    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }

    inline cv::Point3f getColorCV() const {
        return cv::Point3f( color_(0,0), color_(1,0), color_(2,0) );
    }

    inline Vector3d getPosition() const {
        return pos_;
    }

    inline Vector3d getColor() const {
        return color_;
    }

    PointT getPCLPoint();
    
    static LidarPoint::Ptr createLidarPoint();
    static LidarPoint::Ptr createLidarPoint(
        const Vector3d& pos_world, 
        const Vector3d& color,
        bool isAccurate=true
    );

    static LidarPoint::Ptr createLidarPoint (
            const Eigen::Vector4d& pos_world,
            const Vector3d& color,
            bool isAccurate=true
    );

    static LidarPoint::Ptr createLidarPoint (
            const Eigen::Vector4d& pos_world,
            bool isAccurate=true
    );

    bool addObservedFrame(Frame* frame);

    bool isValid();
};

}

#endif // LIDARPOINT_H
