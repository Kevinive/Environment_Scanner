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

#ifndef FEATURE_H
#define FEATURE_H

#include "env_rebuilder/common_include.h"
#include "env_rebuilder/lidarpoint.h"

namespace env_rebuilder
{

class Frame;
class LidarPoint;

class Feature
{
public:
    typedef std::shared_ptr<Feature> Ptr;
    unsigned long      id_;             // ID

    unsigned char        type_;         // 1 for corner, 2 for edge, 3 for LidarPoint
    Vector3d    pos1_;                  // Position in world
    Vector3d    pos2_;                  // 2nd Position in world (for Edge)
    Mat         descriptor1_;           // Descriptor for matching
    Mat         descriptor2_;           // Descriptor for matching (for Edge)

    std::list<Frame*>    observed_frames_;   // key-frames that can observe this point
    std::unordered_map<unsigned long, LidarPoint::Ptr>  lidar_points_;        // all landmarks

private:
    static unsigned long factory_id_;    // factory id

public:
    Feature();
    Feature(
        unsigned long id,
        const unsigned char type,
        const Vector3d& position1,
        const Vector3d& position2 = Vector3d(),
        const Mat& descriptor1=Mat(),
        const Mat& descriptor2=Mat(),
        Frame* frame = nullptr
    );
    
    inline cv::Point3f getCornerFeatureCV() const {
        return cv::Point3f( pos1_(0,0), pos1_(1,0), pos1_(2,0) );
    }
    
    static Feature::Ptr createFeature();
    static Feature::Ptr createCornerFeature(
        const Vector3d& pos1_world,
        const Mat& descriptor1 = Mat(),
        Frame* frame = nullptr);
    static Feature::Ptr createLidarFeature(
            const Vector3d& pos1_world,
            const Mat& descriptor1 = Mat(),
            Frame* frame = nullptr);
};
}

#endif // MAPPOINT_H
