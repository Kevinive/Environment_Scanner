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

#include "env_rebuilder/feature.h"
#include "env_rebuilder/frame.h"

namespace env_rebuilder
{

unsigned long Feature::factory_id_ = 0;

Feature::Feature()
: id_(-1), type_(0), pos1_(Vector3d(0,0,0)), pos2_(Vector3d(0,0,0)), descriptor1_(Mat()), descriptor2_(Mat())
{

}

Feature::Feature( unsigned long id, const unsigned char type, const Vector3d& position1, const Vector3d& position2,
        const Mat& descriptor1, const Mat& descriptor2, Frame* frame
)
: id_(id), type_(type), pos1_(position1), pos2_(position2), descriptor1_(descriptor1), descriptor2_(descriptor2)
{
    observed_frames_.push_back(frame);
}

Feature::Ptr Feature::createFeature()
{
    return Feature::Ptr(
        new Feature( factory_id_++, 0, Vector3d(0,0,0))
    );
}

Feature::Ptr Feature::createCornerFeature(const Vector3d& pos1_world, const Mat& descriptor1, Frame* frame)
{
    return Feature::Ptr(
    new Feature( factory_id_++, 1, pos1_world, Vector3d(0,0,0), descriptor1, Mat(), frame)
    );
}

Feature::Ptr Feature::createLidarFeature(const Vector3d& pos1_world, const Mat& descriptor1, Frame* frame)
{
    return Feature::Ptr(new Feature( factory_id_++, 3, pos1_world, Vector3d(0,0,0), descriptor1, Mat(), frame));
}

}
