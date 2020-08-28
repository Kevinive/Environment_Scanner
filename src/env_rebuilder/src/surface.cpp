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

#include "env_rebuilder/surface.h"
#include "env_rebuilder/point_cloud.h"
#include "env_rebuilder/lidarpoint.h"

namespace env_rebuilder
{
//---------------------------- Surface 部分 ---------------------------------------------
Surface::Surface()
: pointcloud_(nullptr)
{
}

Surface::Surface(PointCloud::Ptr pc){
    pointcloud_ = pc;
}


void Surface::setPointCloud(PointCloud::Ptr pc){
    pointcloud_ = pc;
}

void Surface::rebuildV2(std::vector<LidarPoint::Ptr>& lps, std::vector<Frame::Ptr>& fps){

    const int pointGap = 180;
    queue<Edge::Ptr> loopQue;
    Edge::Ptr lastloopEdge = Edge::createEdge(lps[0], lps[pointGap]);

    unsigned long loopNum = lps.size() - pointGap - 1;
    for(int i=0; i<loopNum; i++){
        Edge::Ptr upEdge;
        if(loopQue.size() >= pointGap){
            upEdge = loopQue.front();
            loopQue.pop();
        }else{
            upEdge = Edge::createEdge(lps[i], lps[i+1]);
        }
        Edge::Ptr downEdge = Edge::createEdge(lps[i + pointGap], lps[i + pointGap + 1]);
        Edge::Ptr longEdge = Edge::createEdge( lps[i], lps[i + pointGap + 1]);
        Edge::Ptr nextEdge = Edge::createEdge( lps[i+1], lps[i + pointGap + 1]);

        SurfaceUnit::Ptr suptUp = SurfaceUnit::createSurfaceUnit(lps[i + pointGap + 1], upEdge);
        SurfaceUnit::Ptr suptDown = SurfaceUnit::createSurfaceUnit(lps[i + pointGap + 1], lastloopEdge);

        downEdge->setSurfaceUnit(suptDown);
        nextEdge->setSurfaceUnit(suptUp);
        longEdge->setSurfaceUnit(suptUp);
        longEdge->setSurfaceUnit(suptDown);
        upEdge->setSurfaceUnit(suptUp);
        lastloopEdge->setSurfaceUnit(suptDown);

        suptUp->insertEdge(longEdge);
        suptUp->insertEdge(nextEdge);
        suptDown->insertEdge(longEdge);
        suptDown->insertEdge(downEdge);

//        insertHalfEdge(ep1);
//        insertHalfEdge(ep2);
//        insertFullEdge(half_edge);

        suptUp->addObservedFrames(fps);
        suptDown->addObservedFrames(fps);

        if(lps[i]->isValid() && lps[i + pointGap + 1]->isValid()){
            if(lps[i+1]->isValid()){
                insertSurfaceUnit(suptUp);
            }
            if(lps[i+pointGap]->isValid()){
                insertSurfaceUnit(suptDown);
            }
        }

        loopQue.push(downEdge);
        lastloopEdge = nextEdge;

    }

    std::unordered_map<unsigned long, env_rebuilder::SurfaceUnit::Ptr >::iterator it;
    for(it=surfaceunits_.begin(); it != surfaceunits_.end(); ){
        env_rebuilder::SurfaceUnit::Ptr su = it->second;
        bool result = su->matchTexture();

        if(!result){
            it = surfaceunits_.erase(it);
        }else{
            cout << "texture pos: (" << su->texture_pixel_point_[0].x() << " "  << su->texture_pixel_point_[0].y() << ") ("
                 << su->texture_pixel_point_[1].x() << " " << su->texture_pixel_point_[1].y() << ") ("
                 << su->texture_pixel_point_[2].x() << " " << su->texture_pixel_point_[2].y() << ") " << endl;
            it++;
        }
    }

}


// 算法存在瑕疵，无法直接使用
void Surface::rebuild(){

    //0 从坐标原点开始生长（选取第一个点）
    std::vector<LidarPoint::Ptr> all_points;
    pointcloud_->getAllPoint(all_points);
    LidarPoint::Ptr firstPoint = nullptr;
    double minDist = 100000.0;
    for(LidarPoint::Ptr lp : all_points){
        double dist = sqrt(lp->pos_.x()*lp->pos_.x() + lp->pos_.y()*lp->pos_.y() + lp->pos_.z()*lp->pos_.z());
        if(dist < minDist){
            minDist = dist;
            firstPoint = lp;
        }
    }

    cout << "First Point Coordinate: ("
        << firstPoint->pos_.x() << ", " << firstPoint->pos_.y() << ", " << firstPoint->pos_.z() << ")" << endl;

    //1 选取第二个点（以第一个点为基准，离其最近的点），并建立第一条边
    LidarPoint::Ptr secondPoint = nullptr;
    minDist = 100000.0;
    for(LidarPoint::Ptr lp : all_points){
        if(lp->id_ == firstPoint->id_) continue;

        double dist = sqrt((lp->pos_.x() - firstPoint->pos_.x())*(lp->pos_.x() - firstPoint->pos_.x())
                           + (lp->pos_.y() - firstPoint->pos_.y())*(lp->pos_.y() - firstPoint->pos_.y())
                           + (lp->pos_.z() - firstPoint->pos_.z())*(lp->pos_.z() - firstPoint->pos_.z()));
        if(dist < minDist){
            minDist = dist;
            secondPoint = lp;
            cout << "New Min Distance: " << minDist << endl;
        }
    }

    cout << "Second Point Coordinate: ("
         << secondPoint->pos_.x() << ", " << secondPoint->pos_.y() << ", " << secondPoint->pos_.z() << ")" << endl;

    Edge::Ptr firstEdge = Edge::createEdge(firstPoint, secondPoint);

    setUsedPoint(firstPoint);
    setUsedPoint(secondPoint);

    //2 选取第三个点（以12为边，用角度判断法找到第三个点），并建立面元
    int step = 0;
    std::vector<LidarPoint::Ptr> candidatePoints;
    double ec_x, ec_y, ec_z;
    ec_x = (firstEdge->point1_->pos_.x() + firstEdge->point2_->pos_.x()) / 2;
    ec_y = (firstEdge->point1_->pos_.y() + firstEdge->point2_->pos_.y()) / 2;
    ec_z = (firstEdge->point1_->pos_.z() + firstEdge->point2_->pos_.z()) / 2;
    pointcloud_->getNearestPoint(ec_x, ec_y, ec_z, candidatePoints);
    while(candidatePoints.size() < 50 && step < 15){
        step++;
        pointcloud_->getNeighborPoint(ec_x, ec_y, ec_z, step, candidatePoints);
    }

    LidarPoint::Ptr lp = nullptr;
    double minAngle = 1.0;
    for(LidarPoint::Ptr i : candidatePoints){
        double vecAngle = getPointAngleCos(i, firstEdge->point1_, firstEdge->point2_);
        if(vecAngle < minAngle){
            minAngle = vecAngle;
            lp = i;
            cout << "New Min Angle: " << minAngle << endl;
        }
    }

    cout << "Third Point Coordinate: ("
         << lp->pos_.x() << ", " << lp->pos_.y() << ", " << lp->pos_.z() << ")" << endl;

    SurfaceUnit::Ptr firstUnit = SurfaceUnit::createSurfaceUnit(lp, firstEdge);
    insertSurfaceUnit(firstUnit);

    //3 将建立13 23边，并将三条边推入半边队列
    Edge::Ptr secondEdge = Edge::createEdge(lp, firstPoint);
    Edge::Ptr thirdEdge = Edge::createEdge(lp, secondPoint);
    setUsedPoint(lp);

    firstUnit->insertEdge(secondEdge);
    firstUnit->insertEdge(thirdEdge);

    firstEdge->setSurfaceUnit(firstUnit);
    secondEdge->setSurfaceUnit(firstUnit);
    thirdEdge->setSurfaceUnit(firstUnit);

    insertHalfEdge(firstEdge);
    insertHalfEdge(secondEdge);
    insertHalfEdge(thirdEdge);

    // 保存某点云
/*
        pcl::PointCloud<pcl::PointXYZ> pc;
        for(LidarPoint::Ptr lp : candidatePoints){
            pcl::PointXYZ point;
            point.x = lp->pos_.x();
            point.y = lp->pos_.y();
            point.z = lp->pos_.z();
            pc.push_back(point);
        }
        pcl::io::savePCDFileBinary("/home/kevin/catkin_workspace/src/env_rebuilder/result/simple_map_test.pcd", pc );
*/
    //4 开始迭代生长！
    int count = 5000;
    while(count--){
        //4.1 从半边队列中取得一条边和该边对应的面元（检查是否为半边，否则跳过），
        //      在离散点中搜寻，找到离散点中夹角最大的那个点（记得处理无符合要求的离散点的情况）
        Edge::Ptr half_edge = half_edges_.front();
        deleteHalfEdge(half_edge);

        LidarPoint::Ptr singlePoint = getNextPoint(half_edge, SurfaceUnit::Ptr(half_edge->surface1_));

        if(singlePoint != nullptr) {
            cout << "New SinglePoint " << singlePoint->id_ << " : ("
                 << singlePoint->pos_.x() << ", " << singlePoint->pos_.y() << ", " << singlePoint->pos_.z() << ")"
                 << endl;
        }

        //4.2 再从半边队列中取得符合要求的半边，与找到的点比较。如果离散点夹角更大则取离散点进行处理；若半边队列中夹角更大则按半边队列进行处理。
        //      （记得处理无符合要求的半边的情况）
        Edge::Ptr secondHalfEdge = getNextEdge(half_edge, SurfaceUnit::Ptr(half_edge->surface1_));

        if(secondHalfEdge != nullptr && singlePoint != nullptr){
            LidarPoint::Ptr sHE_pt;
            if(secondHalfEdge->point1_->id_ == half_edge->point1_->id_ || secondHalfEdge->point1_->id_ == half_edge->point2_->id_){
                sHE_pt = secondHalfEdge->point2_;
            }else {
                sHE_pt = secondHalfEdge->point1_;
            }

            cout << "SecondHalfEdge Point " << sHE_pt->id_ << " : ("
                 << sHE_pt->pos_.x() << ", " << sHE_pt->pos_.y() << ", " << sHE_pt->pos_.z() << ")" << endl;

            double angle1 = getPointAngleCos(singlePoint, half_edge->point1_, half_edge->point2_);
            double angle2 = getPointAngleCos(sHE_pt, half_edge->point1_, half_edge->point2_);
            if(angle1 < angle2){
                secondHalfEdge = nullptr;
            }else{
                singlePoint = nullptr;
            }
        }


        if(secondHalfEdge == nullptr){
            if(singlePoint == nullptr){
                insertHalfEdge(half_edge);
            }else{
                SurfaceUnit::Ptr supt = SurfaceUnit::createSurfaceUnit(singlePoint, half_edge);
                Edge::Ptr ep1 = Edge::createEdge(singlePoint, half_edge->point1_);
                Edge::Ptr ep2 = Edge::createEdge(singlePoint, half_edge->point2_);

                half_edge->setSurfaceUnit(supt);

                supt->insertEdge(ep1);
                supt->insertEdge(ep2);

                ep1->setSurfaceUnit(supt);
                ep2->setSurfaceUnit(supt);

                insertHalfEdge(ep1);
                insertHalfEdge(ep2);
                insertFullEdge(half_edge);
                insertSurfaceUnit(supt);
                setUsedPoint(singlePoint);
            }
        }else{
            LidarPoint::Ptr base_ptr, p1, p2;
            if(secondHalfEdge->point1_->id_ == half_edge->point1_->id_){
                base_ptr = secondHalfEdge->point1_;
                p1 = half_edge->point2_;
                p2 = secondHalfEdge->point2_;
            }else if(secondHalfEdge->point1_->id_ == half_edge->point2_->id_){
                base_ptr = secondHalfEdge->point1_;
                p1 = half_edge->point1_;
                p2 = secondHalfEdge->point2_;
            }else if(secondHalfEdge->point2_->id_ == half_edge->point1_->id_){
                base_ptr = secondHalfEdge->point2_;
                p1 = half_edge->point2_;
                p2 = secondHalfEdge->point1_;
            }else if(secondHalfEdge->point2_->id_ == half_edge->point2_->id_){
                base_ptr = secondHalfEdge->point2_;
                p1 = half_edge->point1_;
                p2 = secondHalfEdge->point1_;
            }

            SurfaceUnit::Ptr supt = SurfaceUnit::createSurfaceUnit(p2, half_edge);
            Edge::Ptr ep1 = Edge::createEdge(p1, p2);

            half_edge->setSurfaceUnit(supt);
            secondHalfEdge->setSurfaceUnit(supt);

            supt->insertEdge(ep1);
            supt->insertEdge(secondHalfEdge);

            ep1->setSurfaceUnit(supt);
            secondHalfEdge->setSurfaceUnit(supt);

            insertHalfEdge(ep1);
            insertFullEdge(secondHalfEdge);
            insertFullEdge(half_edge);
            insertSurfaceUnit(supt);
        }
    }



}

LidarPoint::Ptr Surface::getNextPoint(Edge::Ptr edge, SurfaceUnit::Ptr su){

    //0 判断：edge为su的一条边&&edge是半边而不是全边（否则退出）
    if(edge->edgetype_ >= 2 || !isInSurfaceUnit(edge, su)) return nullptr;

    //1 调出edge中点坐标范围内的所有点
    int step = 0;
    std::vector<LidarPoint::Ptr> candidatePoints;
    double ec_x, ec_y, ec_z;
    ec_x = (edge->point1_->pos_.x() + edge->point2_->pos_.x()) / 2;
    ec_y = (edge->point1_->pos_.y() + edge->point2_->pos_.y()) / 2;
    ec_z = (edge->point1_->pos_.z() + edge->point2_->pos_.z()) / 2;
    pointcloud_->getNearestPoint(ec_x, ec_y, ec_z, candidatePoints);
    while(candidatePoints.size() < 50 && step < 15){
        step++;
        pointcloud_->getNeighborPoint(ec_x, ec_y, ec_z, step, candidatePoints);
    }

    //2 将这些点投影到su平面上，并判断：1 该点是否不与su的第三个点在同一侧 2 计算夹角，是否为最大（记录最大点与最大夹角）
    Eigen::Isometry3d tfMatrix = calcTransferMatrix(edge, su);
    LidarPoint::Ptr lp = nullptr;
    double minAngle = 1.0;
    for(LidarPoint::Ptr i : candidatePoints){
        if(isPointUsed(i)) continue;
        Eigen::Vector2d projected_point = project2Surface(i->pos_, tfMatrix);
        if(projected_point.y() > 0.0) continue;
        double vecAngle = getPointAngleCos(i, edge->point1_, edge->point2_);
        if(vecAngle < minAngle){
            minAngle = vecAngle;
            lp = i;
        }
    }

    //3 返回该点
    return lp;
}

Edge::Ptr Surface::getNextEdge(Edge::Ptr edge, SurfaceUnit::Ptr su){

    //0 判断：edge为su的一条边&&edge是半边而不是全边（否则退出）
    if(edge->edgetype_ >= 2 || !isInSurfaceUnit(edge, su)) return nullptr;

    //1 遍历所有半边
    Edge::Ptr minEdge = nullptr;
    double minCos = 1.0;
    for(std::list<Edge::Ptr>::iterator iter = half_edges_.begin(); iter != half_edges_.end() ;iter++)
    {
        // 1.1 若半边的端点没有和edge端点相同，则跳过此半边
        LidarPoint::Ptr p_com, p_1, p_2;
        p_com = nullptr;
        if((*iter)->point1_->id_ == edge->point1_->id_ && (*iter)->point2_->id_ != edge->point2_->id_){
            p_com = (*iter)->point1_;
            p_1 = edge->point2_;
            p_2 = (*iter)->point2_;
        }else if((*iter)->point1_->id_ == edge->point2_->id_ && (*iter)->point2_->id_ != edge->point1_->id_){
            p_com = (*iter)->point1_;
            p_1 = edge->point1_;
            p_2 = (*iter)->point2_;
        }else if((*iter)->point2_->id_ == edge->point1_->id_ && (*iter)->point1_->id_ != edge->point2_->id_){
            p_com = (*iter)->point2_;
            p_1 = edge->point2_;
            p_2 = (*iter)->point1_;
        }else if((*iter)->point2_->id_ == edge->point2_->id_ && (*iter)->point1_->id_ != edge->point1_->id_){
            p_com = (*iter)->point2_;
            p_1 = edge->point1_;
            p_2 = (*iter)->point1_;
        }
        if(p_com == nullptr) continue;

        // 1.2 若半边的一个端点相同，则开始判断
        // 记录选择的那个半边的另一个点，交叉判断是否在另一边（如果是则记录下此半边的点的夹角，作为Min，继续搜寻）

        Eigen::Isometry3d tf1 = calcTransferMatrix(edge, SurfaceUnit::Ptr(edge->surface1_));
        Eigen::Vector2d p_2_pj = project2Surface(p_2->pos_, tf1);
        if(p_2_pj.y() >= 0.0) continue;

        Eigen::Isometry3d tf2 = calcTransferMatrix(*iter, SurfaceUnit::Ptr((*iter)->surface1_));
        Eigen::Vector2d p_1_pj = project2Surface(p_1->pos_, tf2);
        if(p_1_pj.y() >= 0.0) continue;

        double angleCos = getPointAngleCos(p_2, p_1, p_com);
        if(angleCos < minCos){
            minCos = angleCos;
            minEdge = *iter;
        }

    }

    //3 返回该半边。
    return minEdge;
}

bool Surface::isInSurfaceUnit(Edge::Ptr edge, SurfaceUnit::Ptr su){
    return su->isEdgeInSU(edge);
}

double Surface::getPointAngleCos(LidarPoint::Ptr base_p, LidarPoint::Ptr p1, LidarPoint::Ptr p2){
    Vector3d v1 = base_p->getPosition() - p1->getPosition();
    Vector3d v2 = base_p->getPosition() - p2->getPosition();
    double angle_cos = v1.dot(v2)/v1.norm()/v2.norm();
    return angle_cos;
}

Eigen::Vector2d Surface::project2Surface(Eigen::Vector3d pt, Eigen::Isometry3d tf){
    Eigen::Vector3d result = tf*pt;
    return Eigen::Vector2d(result.x(), result.y());
}

Eigen::Isometry3d Surface::calcTransferMatrix(Edge::Ptr ep, SurfaceUnit::Ptr supt){
    Eigen::Vector3d origin = ep->point1_->pos_;
    Eigen::Vector3d x_vec = ep->point2_->pos_ - ep->point1_->pos_;
    Eigen::Vector3d y_vec;
    for(LidarPoint::Ptr lp : supt->points_){
        if(lp->id_ != ep->point1_->id_ && lp->id_ != ep->point2_->id_){
            y_vec = lp->pos_ - ep->point1_->pos_;
            break;
        }
    }

    x_vec.normalize();
    y_vec = y_vec - (x_vec * x_vec.dot(y_vec));
    y_vec.normalize();
    Eigen::Vector3d z_vec = x_vec.cross(y_vec);
    z_vec.normalize();

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    rot <<  x_vec.x(), y_vec.x(), z_vec.x(),
            x_vec.y(), y_vec.y(), z_vec.y(),
            x_vec.z(), y_vec.z(), z_vec.z();

    tf.rotate(rot);
    tf.pretranslate(origin);

    return tf.inverse();

}

bool Surface::insertSurfaceUnit(SurfaceUnit::Ptr supt){
    if( isInSurfaceUnitList(supt) ) return false;

    surfaceunits_.insert(make_pair(supt->id_, supt));
    return true;
}

bool Surface::insertHalfEdge(Edge::Ptr ep){
    if(!ep->isHalfEdge()) return false;

    if( isInHalfEdgeList(ep) ) return false;
    half_edges_.push_back(ep);
    return true;
}

bool Surface::deleteHalfEdge(Edge::Ptr ep){
    std::list<Edge::Ptr>::iterator iter;
    iter = std::find(half_edges_.begin(), half_edges_.end(), ep);
    if(iter != half_edges_.end()){
        half_edges_.erase(iter);
        return true;
    }
    return false;
}

bool Surface::insertFullEdge(Edge::Ptr ep){
    if(!ep->isFullEdge()) return false;

    if( isInFullEdgeList(ep) ) return false;

    full_edges_.insert(make_pair(ep->id_, ep));
    return true;
}

void Surface::setUsedPoint(LidarPoint::Ptr lp){
    if ( used_points_.find(lp->id_) == used_points_.end() ){
        used_points_.insert(make_pair(lp->id_, lp));

    }
}

bool Surface::isInHalfEdgeList(Edge::Ptr ep){
    std::list<Edge::Ptr>::iterator iter;
    iter = std::find(half_edges_.begin(), half_edges_.end(), ep);
    if(iter != half_edges_.end()) return true;
    else return false;
}

bool Surface::isInFullEdgeList(Edge::Ptr ep){
    if ( full_edges_.find(ep->id_) == full_edges_.end() )
    {
        return false;
    }
    return true;
}

bool Surface::isInSurfaceUnitList(SurfaceUnit::Ptr supt){
    if ( surfaceunits_.find(supt->id_) == surfaceunits_.end() )
    {
        return false;
    }
    return true;
}


bool Surface::isPointUsed(LidarPoint::Ptr lp){
    if ( used_points_.find(lp->id_) == used_points_.end() ){
        return false;
    }
    return true;
}

//---------------------------- Edge 部分 ---------------------------------------------

/*
 * 构造函数们
 * 构造方式：
 * 1 无参数
 * 2 只用ID
 * 3 用ID和2个点
 */
Edge::Edge()
: id_(0), point1_(nullptr), point2_(nullptr), edgetype_(0), surface1_(nullptr), surface2_(nullptr)
{}
Edge::Edge(unsigned long id)
: id_(id), point1_(nullptr), point2_(nullptr), edgetype_(0), surface1_(nullptr), surface2_(nullptr)
{}
Edge::Edge(unsigned long id, LidarPoint::Ptr point1, LidarPoint::Ptr point2)
: id_(id), point1_(point1), point2_(point2), edgetype_(0), surface1_(nullptr), surface2_(nullptr)
{}

/*
 * 辅助构造函数：自动按顺序填充id
 * 构造方式：
 * 1 无参数
 * 2 使用2个雷达数据创建
 */
Edge::Ptr Edge::createEdge(){
    return Edge::Ptr(
        new Edge( factory_id_++ )
    );
}
Edge::Ptr Edge::createEdge(LidarPoint::Ptr point1, LidarPoint::Ptr point2){
    return Edge::Ptr(
            new Edge( factory_id_++ , point1, point2)
    );
}

bool Edge::setSurfaceUnit(std::shared_ptr<SurfaceUnit> supt){
    if(edgetype_>=2) return false;
    if(edgetype_ == 0) surface1_ = supt;
    else surface2_ = supt;
    edgetype_++;
    return true;
}

/*
 * 判断函数：点是否在边中
 */
bool Edge::isInEdge(LidarPoint::Ptr lp){
    if(lp->id_ == point1_->id_ || lp->id_ == point2_->id_) return true;
    return false;
}

/*
 * 判断函数：是否为空边
 */
bool Edge::isEmptyEdge(){
    return (edgetype_==0);
}

/*
 * 判断函数：是否为半边
 */
bool Edge::isHalfEdge(){
    return (edgetype_==1);
}

/*
 * 判断函数：是否为全边
 */
bool Edge::isFullEdge(){
    return (edgetype_==2);
}

unsigned long Edge::factory_id_ = 0;

//---------------------------- Surface Unit 部分 ---------------------------------------------
/*
 * 构造函数们
 * 构造方式：
 * 1 无参数
 * 2 只用ID
 * 3 用ID和几个点（最多会被采纳3个）
 * 4 用ID和一个点
 */
SurfaceUnit::SurfaceUnit()
: id_(-1), centralPos_(Vector3d(0,0,0)), isWellDefined_(false)
{

}

SurfaceUnit::SurfaceUnit(unsigned long id)
: id_(id), centralPos_(Vector3d(0,0,0)), isWellDefined_(false)
{
}

SurfaceUnit::SurfaceUnit (unsigned long id, const std::vector<LidarPoint::Ptr>& points)
: id_(id)
{
    for(LidarPoint::Ptr lp : points){
        if(points_.size() >= 3) break;
        else if(points_.size() >= 1 && isPointInSU(lp)){
            continue;
        }
        points_.push_back(lp);
    }

    if(points_.size() == 3){
        isWellDefined_ = true;
        calcCG();
    }else{
        isWellDefined_ = false;
    }


}

SurfaceUnit::SurfaceUnit(unsigned long id, LidarPoint::Ptr point, Edge::Ptr edge)
:id_(id), centralPos_(Vector3d(0,0,0))
{
    points_.push_back(point);
    if(!isPointInSU(edge->point1_)) points_.push_back(edge->point1_);
    if(!isPointInSU(edge->point2_)) points_.push_back(edge->point2_);
    edges_.push_back(edge);
    if(points_.size() == 3){
        isWellDefined_ = true;
        calcCG();
    }else{
        isWellDefined_ = false;
    }
}

/*
 * 辅助构造函数：自动按顺序填充id
 * 构造方式：
 * 1 无参数
 * 2 使用几个雷达数据创建
 * 3 使用一个雷达数据创建
 *
 */
SurfaceUnit::Ptr SurfaceUnit::createSurfaceUnit()
{
    return SurfaceUnit::Ptr(
            new SurfaceUnit( factory_id_++)
    );
}

SurfaceUnit::Ptr SurfaceUnit::createSurfaceUnit ( const std::vector<LidarPoint::Ptr>& points )
{
    return SurfaceUnit::Ptr(
            new SurfaceUnit( factory_id_++, points )
    );
}

SurfaceUnit::Ptr SurfaceUnit::createSurfaceUnit ( LidarPoint::Ptr point, Edge::Ptr edge )
{
    return SurfaceUnit::Ptr(
            new SurfaceUnit( factory_id_++, point, edge )
    );
}


/*
 * Function: 插入已经观察到的图像
 * 非公开函数，供addObservedFrames()调用
 */
void SurfaceUnit::insertObservedFrame ( Frame::Ptr frame )
{
    if ( !isFrameInSU(frame) )
    {
        observed_frame_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        observed_frame_[ frame->id_ ] = frame;
    }
}

/*
 * Function: 插入面元包含的点
 */
bool SurfaceUnit::insertPoint ( LidarPoint::Ptr lp )
{
    if(isWellDefined_) return false;
    if(!isPointInSU(lp)){
        points_.push_back(lp);
    }else{
        return false;
    }

    if(points_.size() == 3)
        isWellDefined_ = true;

    return true;
}

/*
 * Function: 插入面元包含的边界
 */
bool SurfaceUnit::insertEdge(Edge::Ptr ep){
    if(edges_.size() >= 3) return false;
    if(isEdgeInSU(ep)){
        return false;
    }

    edges_.push_back(ep);
    return true;
}

/*
 * Function: 计算面元的CG
 */
void SurfaceUnit::calcCG(){
    if(!isWellDefined_){
        centralPos_ = Vector3d(0,0,0);
        return;
    }
    Vector3d cg(0.0, 0.0, 0.0);
    for(LidarPoint::Ptr lp : points_){
        cg += lp->pos_;
    }
    centralPos_ = cg / 3;
}

/*
 * Function: 传入一串图像，识别面元是否在当前图像中被观察到
 * 只有面元的三个点同时在图像内才会判断为图像观察到了这个面元。
 */
int SurfaceUnit::addObservedFrames(vector<Frame::Ptr>& frames){
    int addFramesCount = 0;

    if(!isWellDefined_){
        return 0;
    }

    for(Frame::Ptr fp : frames){
        bool isInFrame = true;
        for(LidarPoint::Ptr lp : points_){
            Vector3d p_cam = fp->camera_->world2camera(lp->pos_, fp->T_c_w_);
            if(p_cam.x() < 0.1){
                isInFrame = false;
                break;
            }
            Vector2d p_pixel = fp->camera_->camera2pixel(p_cam);
            if(!(p_pixel.x() > 0.0 && p_pixel.x() < fp->camera_->width_ && p_pixel.y() > 0.0 && p_pixel.y() < fp->camera_->hight_)){
                isInFrame = false;
                break;
            }
        }

        // 如果三个点都在画面内，那么CG一定也在画面内
        if(isInFrame){
            insertObservedFrame (fp);
            addFramesCount++;
        }

    }

    return addFramesCount;
}

// TODO 校验此算法的有效性
/*
 * Function: 传入一串图像，识别面元是否在当前图像中被观察到
 * 只有面元的三个点同时在图像内才会判断为图像观察到了这个面元。
 */
bool SurfaceUnit::matchTexture(){
    bool isMatched = false;

    if(!isWellDefined_ || observed_frame_.size() == 0){
        return false;
    }

    calcCG();

    unordered_map<unsigned long, Frame::Ptr>::iterator fpit = observed_frame_.begin();
    double minDist2Central = 1000000.0;

    while(fpit != observed_frame_.end()){
        Frame::Ptr fp = fpit->second;
        Vector2d centralPoint = fp->camera_->world2pixel(centralPos_, fp->T_c_w_);
        double dist2Central = sqrt(
                (centralPoint.x() - fp->camera_->cx_)*(centralPoint.x() - fp->camera_->cx_) +
                (centralPoint.y() - fp->camera_->cy_)*(centralPoint.y() - fp->camera_->cy_));
        if(dist2Central < minDist2Central){
            minDist2Central = dist2Central;
            texture_frame_ = fp;
            isMatched = true;
        }

        fpit++;
    }

    if(isMatched){
        texture_pixel_point_.clear();
        for(LidarPoint::Ptr lp : points_){
            texture_pixel_point_.push_back(
                    texture_frame_->camera_->world2pixel(lp->pos_, texture_frame_->T_c_w_)
                    );
        }
    }

    return isMatched;
}

bool SurfaceUnit::isPointInSU(LidarPoint::Ptr lp){
    for(LidarPoint::Ptr testlp : points_){
        if(lp->id_ == testlp->id_) return true;
    }
    return false;
}

bool SurfaceUnit::isEdgeInSU(Edge::Ptr ep){
    for(auto testep : edges_){
        if(ep->id_ == testep->id_) return true;
    }
    return false;
}

bool SurfaceUnit::isFrameInSU(Frame::Ptr fp){
    if ( observed_frame_.find(fp->id_) == observed_frame_.end() )
    {
        return false;
    }
    return true;
}


unsigned long SurfaceUnit::factory_id_ = 0;

}
