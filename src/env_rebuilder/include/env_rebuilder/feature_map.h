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

#ifndef FEATURE_MAP_H
#define FEATURE_MAP_H

#include "env_rebuilder/common_include.h"
#include "env_rebuilder/feature.h"

namespace env_rebuilder
{

class Feature;

class FeatureMap
{
public:
    typedef std::shared_ptr<FeatureMap> Ptr;
    std::vector< Feature::Ptr >  features_;        // all landmarks

    FeatureMap() {}

    void insertFeature( Feature::Ptr feature );
    void deleteFeature( unsigned long id );

    Feature::Ptr iterator_getNextFeature();

    unsigned int merge_features(unsigned int goalNum);

private:
    int DescriptorDistance (const cv::Mat &a, const cv::Mat &b);

};
}

#endif // FEATURE_MAP_H
