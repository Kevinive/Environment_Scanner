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

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "env_rebuilder/common_include.h"
#include "env_rebuilder/frame.h"
#include "env_rebuilder/lidarpoint.h"

namespace env_rebuilder
{

class LidarPoint;

/*
 * 用来分空间存储雷达点的类
 */

template <typename T>
class SpaceHashMap
{
public:
    typedef std::shared_ptr<SpaceHashMap<T>> Ptr;
    double max_X_, max_Y_, max_Z_;
    double unit_scale_;
    int x_count_, y_count_, z_count_;
    bool isInitialized_;

    std::unordered_map<unsigned long, std::vector<T>>  map_;        // all landmarks

    SpaceHashMap();
    SpaceHashMap(double max_x, double max_y, double max_z, double unit_scale);
    bool setParams(double max_x, double max_y, double max_z, double unit_scale);

    unsigned long save(const T obj, double x, double y, double z);
    void get(double x, double y, double z, std::vector<T>& result);
    void getNeighbor(double x, double y, double z, int step, std::vector<T>& result);
    void getAllObject(std::vector<T>& result);

private:
    void xyz2xyz_unit(double x, double y, double z, int* result);
    unsigned long xyz2hash(double x, double y, double z);
    unsigned long xyz_unit2hash(int x, int y, int z);
    void hash2xyz_unit(unsigned long hash, int* result);
    void hash2xyz(unsigned long hash, double* result);

    bool isInMap(unsigned long hash);
};


class PointCloud
{
public:
    typedef std::shared_ptr<PointCloud> Ptr;
    SpaceHashMap<LidarPoint::Ptr> point_map_;        // all landmarks
    double x_offset_, y_offset_, z_offset_;
    double x_size_, y_size_, z_size_;
    double unit_scale_;

    PointCloud(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double unit_scale);

    void insertLidarPoint( LidarPoint::Ptr lidar_point );
    void insertLidarPoint( std::vector<LidarPoint::Ptr>& lpvec );

    void getNearestPoint( double x, double y, double z , std::vector<LidarPoint::Ptr>& result);
    void getNeighborPoint( double x, double y, double z , int step, std::vector<LidarPoint::Ptr>& result);
    void getAllPoint(std::vector<LidarPoint::Ptr>& result);

    static PointCloud::Ptr createPointCloud(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double unit_scale);
    static PointCloud::Ptr createPointCloud(std::vector<LidarPoint::Ptr>& lpvec, double unit_scale);
};


/******************************** Impl of SpaceHashMap *************************************/
template <typename T>
SpaceHashMap<T>::SpaceHashMap()
: max_X_(0), max_Y_(0), max_Z_(0), unit_scale_(1.0), isInitialized_(false)
{}

template <typename T>
SpaceHashMap<T>::SpaceHashMap(double max_x, double max_y, double max_z, double unit_scale)
: max_X_(max_x), max_Y_(max_y), max_Z_(max_z), unit_scale_(unit_scale), isInitialized_(false)
{
    if(max_x > 0.0 && max_y > 0.0 && max_z > 0.0 && unit_scale > 0.0) {
        x_count_ = int(max_x / unit_scale) + 1;
        y_count_ = int(max_y / unit_scale) + 1;
        z_count_ = int(max_z / unit_scale) + 1;
        /*cout << "HashMap Initialized with maxX: " << to_string(max_X_) << "\tmaxY: " << to_string(max_Y_) << "\tmaxZ: "
             << to_string(max_Z_)
             << "\tx_count: " << x_count_ << "\ty_count: " << y_count_ << "\tz_count: " << z_count_ << endl;
             */
        isInitialized_ = true;
    }
}

template <typename T>
bool SpaceHashMap<T>::setParams(double max_x, double max_y, double max_z, double unit_scale){
    max_X_ = max_x;
    max_Y_ = max_y;
    max_Z_ = max_z;
    unit_scale_ = unit_scale;
    isInitialized_ = false;
    map_.clear();

    if(max_x > 0.0 && max_y > 0.0 && max_z > 0.0 && unit_scale > 0.0) {
        x_count_ = int(max_x / unit_scale) + 1;
        y_count_ = int(max_y / unit_scale) + 1;
        z_count_ = int(max_z / unit_scale) + 1;
        /*cout << "HashMap Initialized with maxX: " << to_string(max_X_) << "\tmaxY: " << to_string(max_Y_) << "\tmaxZ: "
             << to_string(max_Z_)
             << "\tx_count: " << x_count_ << "\ty_count: " << y_count_ << "\tz_count: " << z_count_ << endl;
             */
        isInitialized_ = true;
    }
    return isInitialized_;
}

template <typename T>
unsigned long SpaceHashMap<T>::save(const T obj, double x, double y, double z){
    if(!isInitialized_) return x_count_*y_count_*z_count_;

    if(x > max_X_ || y > max_Y_ || z > max_Z_) return x_count_*y_count_*z_count_;

    unsigned long hash = xyz2hash(x, y, z);
    //cout << "Point(" << to_string(x) << ", " << to_string(y) << ", "<< to_string(z) << ") saved in Hash: " << hash << endl;
    if ( !isInMap(hash) )
    {
        std::vector<T> vt;
        vt.push_back(obj);
        map_.insert(make_pair(hash, vt));
        //cout << "Hash " << hash << " index not exist, create one" << endl;
    }
    else
    {
        map_[hash].push_back(obj);
        //cout << "add Point to existed hash list." << endl;
    }
}

template <typename T>
void SpaceHashMap<T>::get(double x, double y, double z, std::vector<T>& result){
    if(!isInitialized_) return;

    if(x > max_X_ || y > max_Y_ || z > max_Z_) return;

    unsigned long hash = xyz2hash(x, y, z);
    if ( !isInMap(hash) )
    {
        //cout << "the block " << hash << " is blank, return nothing." << endl;
        return;
    }
    //cout << "have the result of " << map_[hash].size() << " item(s)." << endl;
    result.insert(result.end(), map_[hash].begin(), map_[hash].end());
}

template <typename T>
void SpaceHashMap<T>::getNeighbor(double x, double y, double z, int step, std::vector<T>& result){
    if(!isInitialized_) return;

    if(x > max_X_ || y > max_Y_ || z > max_Z_ || step <= 0) return;

    int xyz[3] = {0};
    xyz2xyz_unit(x, y, z, xyz);

    int x_start = max(0, xyz[0]-step);
    int x_end = min(x_count_, xyz[0]+step+1);
    int y_start = max(0, xyz[1]-step);
    int y_end = min(y_count_, xyz[1]+step+1);
    int z_start = max(0, xyz[2]-step);
    int z_end = min(z_count_, xyz[2]+step+1);

    for(int i=x_start; i<x_end; i++){
        for(int j=y_start; j<y_end; j++){
            for(int k=z_start; k<z_end; k++){
                if(k==z_start || k==xyz[2]+step || j==y_start || j==xyz[1]+step || i==x_start || i==xyz[0]+step){
                    unsigned long hash = xyz_unit2hash(i, j, k);
                    //cout << "the result of " << i << " " << j << " " << k << " is accepted." << endl;
                    if ( isInMap(hash) )
                    {
                        result.insert(result.end(), map_[hash].begin(), map_[hash].end());
                    }
                }else{
                    //cout << "the result of " << i << " " << j << " " << k << " is denied." << endl;
                }
            }
        }
    }

}
/*
 * 获得所有对象
 */
template <typename T>
void SpaceHashMap<T>::getAllObject(std::vector<T>& result){
    if(!isInitialized_) return;

    typename std::unordered_map<unsigned long, std::vector<T>>::iterator it = map_.begin();
    while(it != map_.end()){
        result.insert(result.end(), it->second.begin(), it->second.end());
        it++;
    }
}

/*
 * XYZ坐标  到  XYZ单位坐标
 */
template <typename T>
void SpaceHashMap<T>::xyz2xyz_unit(double x, double y, double z, int* result){
    result[0] = int(x / unit_scale_);
    result[1] = int(y / unit_scale_);
    result[2] = int(z / unit_scale_);
}

/*
 * XYZ坐标  到  哈希值的转换函数
 */
template <typename T>
unsigned long SpaceHashMap<T>::xyz2hash(double x, double y, double z){
    return (unsigned long)(int(z/unit_scale_)*y_count_*x_count_ + int(y/unit_scale_)*x_count_ + int(x/unit_scale_));
}

/*
 * XYZ单位坐标  到  哈希值的转换函数
 */
template <typename T>
unsigned long SpaceHashMap<T>::xyz_unit2hash(int x, int y, int z){
    return (unsigned long)(z*y_count_*x_count_ + y*x_count_ + x);
}

/*
 * 哈西值  到  XYZ单位坐标的转换函数
 */
template <typename T>
void SpaceHashMap<T>::hash2xyz_unit(unsigned long hash, int* result){
    result[2] = int(hash / x_count_ / y_count_);
    result[1] = int((hash%(x_count_*y_count_))/x_count_);
    result[0] = int(hash % x_count_);
}

/*
 * 哈西值  到  XYZ坐标的转换函数
 */
template <typename T>
void SpaceHashMap<T>::hash2xyz(unsigned long hash, double* result){
    result[2] = int(hash / x_count_ / y_count_)*unit_scale_;
    result[1] = int((hash%(x_count_*y_count_))/x_count_)*unit_scale_;
    result[0] = int(hash % x_count_)*unit_scale_;
}

/*
 * 查找map中是否有相应HASH值的项
 */
template <typename T>
bool SpaceHashMap<T>::isInMap(unsigned long hash){
    if ( map_.find(hash) == map_.end() )
    {
        return false;
    }
    else
    {
        return true;
    }
}


}

#endif // POINT_CLOUD_H
