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

#include "env_rebuilder/feature_map.h"


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace env_rebuilder
{

void FeatureMap::insertFeature( Feature::Ptr feature )
{
    //cout<<"Feature pool size = "<<features_.size()<<endl;
    features_.push_back(feature);
}

void FeatureMap::deleteFeature( unsigned long id )
{
    //cout<<"Feature pool size = "<<features_.size()<<endl;
    for(auto it = features_.begin(); it != features_.end(); it++){
        if(it->get()->id_ == id){
            it = features_.erase(it);
            if(it == features_.end()) break;
        }
    }
}



Feature::Ptr FeatureMap::iterator_getNextFeature(){
    static vector<Feature::Ptr>::iterator it = features_.begin();
    Feature::Ptr fptr;
    if(it != features_.end()){
        fptr = *it;
        it++;
    }else{
        fptr = *it;
        it = features_.begin();
    }
    return fptr;
}

/*
 * Function: merge_features
 * goalNum： 没有作用
 * 主要功能：合并特征点云，压缩特征点所占用的空间。
 *          暴力计算所有特征点之间的汉明距离和坐标距离，当两者都小于一个阈值时认为两个特征点为同一点，从而进行合并。
 */
// TODO 将两特征点为同一点的判断标准细化，浮动化，以改进精度（eg 特征点距离传感器越近，坐标距离阈值应越小）
// TODO 利用上goalNum,只合并最相似的特征点，合并到goalNum即可，以保留可观数量的特征点（但特征点的真实数量我们不得而知，此算法是否有效还待商榷）
unsigned int FeatureMap::merge_features(unsigned int goalNum){
    // 1st: 遍历特征，确定特征描述子的最大最小距离和特征点之间的最大最小距离；
    // 2nd: 先定目标为原先点数的一半，运行标准K-MEAN程序，看方差能否接受
    // 3rd： 若可以接受则继续缩减点数，直到目标点数或方差不能接受

    if(goalNum >= features_.size() || features_.size() == 0) return features_.size();

    char changeFlag = 0;
    do {
        changeFlag = 0;
        cv::Mat_<float> distanceMat(features_.size(), features_.size());
        double min_hamming_dist = 10000, max_hamming_dist = 0;
        double min_real_dist = 10000, max_real_dist = 0;
        for (int i = 0; i < features_.size(); i++) {
            for (int j = i + 1; j < features_.size(); j++) {
                distanceMat(i, j) = DescriptorDistance(features_[i]->descriptor1_, features_[j]->descriptor1_);
                distanceMat(j, i) = sqrt(
                        (features_[i]->pos1_.x() - features_[j]->pos1_.x()) *
                        (features_[i]->pos1_.x() - features_[j]->pos1_.x())
                        + (features_[i]->pos1_.y() - features_[j]->pos1_.y()) *
                          (features_[i]->pos1_.y() - features_[j]->pos1_.y())
                        + (features_[i]->pos1_.z() - features_[j]->pos1_.z()) *
                          (features_[i]->pos1_.z() - features_[j]->pos1_.z()));
                if(distanceMat(i, j) < 8.0 && distanceMat(j, i) < 30.0) {
                    cout << "The Hamming Distance between " << i << " and " << j << " is: "<< distanceMat(i, j) << endl;
                    cout << "The Real Distance between " << i << " and " << j << " is: "<< distanceMat(j, i) << endl;
                }
                if (distanceMat(i, j) < min_hamming_dist) min_hamming_dist = distanceMat(i, j);
                if (distanceMat(i, j) > max_hamming_dist) max_hamming_dist = distanceMat(i, j);
                if (distanceMat(j, i) < min_real_dist) min_real_dist = distanceMat(j, i);
                if (distanceMat(j, i) > max_real_dist) max_real_dist = distanceMat(j, i);
            }
        }

        cout << "The min Hamming Distance is: " << min_hamming_dist << endl;
        cout << "The max Hamming Distance is: " << max_hamming_dist << endl;
        cout << "The min Real Distance is: " << min_real_dist << endl;
        cout << "The max Real Distance is: " << max_real_dist << endl;

        float hamming_threshold = max ( 2 * min_hamming_dist, 30.0 );
        float real_threshold = 8.0;
        vector<unsigned long> invalidFeatures;
        for (int i = 0; i < features_.size(); i++) {

            // 判断第I个元素是否可作为种子
            int k1;
            for(k1 = 0; k1 < invalidFeatures.size(); k1++){
                if(invalidFeatures[k1] == features_[i]->id_)
                    break;
            }
            if (k1 != invalidFeatures.size())
                continue;

            int numCount = 1;
            for (int j = i + 1; j < features_.size(); j++) {
                int k2;
                for(k2 = 0; k2 < invalidFeatures.size(); k2++){
                    if(invalidFeatures[k2] == features_[j]->id_)
                        break;
                }

                if (k2 != invalidFeatures.size())
                    continue;

                if (distanceMat(i, j) < hamming_threshold && distanceMat(j, i) < real_threshold) {
                    features_[i]->pos1_ += features_[j]->pos1_;
                    numCount++;
                    invalidFeatures.push_back(features_[j]->id_);
                    changeFlag = 1;
                }
            }
            features_[i]->pos1_ /= numCount;
        }

        for (unsigned long id : invalidFeatures) {
            deleteFeature(id);
        }

        cout << "Current Feature Num: " << features_.size() << endl;

    }while(changeFlag);

    return features_.size();
}

// 计算 两个点之间的HAMMING DISTANCE
int FeatureMap::DescriptorDistance (const cv::Mat &a, const cv::Mat &b) {
    // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();
    int loopnum = min(a.cols*a.elemSize()/4, b.cols*b.elemSize()/4);
    int dist = 0;
    for(int i = 0; i < loopnum; i++, pa++, pb++) {
        unsigned int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    return dist;
}

}
