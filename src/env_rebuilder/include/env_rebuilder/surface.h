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

#ifndef SURFACE_H
#define SURFACE_H

#include "env_rebuilder/common_include.h"
#include "env_rebuilder/frame.h"
#include "env_rebuilder/lidarpoint.h"
#include "env_rebuilder/point_cloud.h"

namespace env_rebuilder
{
    
class Frame;
class LidarPoint;

class SurfaceUnit;

class Edge
{
public:
    typedef std::shared_ptr<Edge>   Ptr;
    unsigned long                   id_;                        // ID
    LidarPoint::Ptr                 point1_;                    // CornerPoints
    LidarPoint::Ptr                 point2_;                    // CornerPoints
    std::shared_ptr<SurfaceUnit>    surface1_;                    // CornerPoints
    std::shared_ptr<SurfaceUnit>    surface2_;                    // CornerPoints

    char edgetype_;         // 0 for empty; 1 for half edge(part of 1 surfaceunit); 2 for full edge(a edge of 2 surfaceunit)

private:
    static unsigned long factory_id_;    // factory id

public:
    Edge();
    Edge(unsigned long id);
    Edge(unsigned long id, LidarPoint::Ptr point1, LidarPoint::Ptr point2);
    static Edge::Ptr createEdge();
    static Edge::Ptr createEdge(LidarPoint::Ptr point1, LidarPoint::Ptr point2);

    bool setSurfaceUnit(std::shared_ptr<SurfaceUnit> supt);
    bool isInEdge(LidarPoint::Ptr lp);
    bool isEmptyEdge();
    bool isHalfEdge();
    bool isFullEdge();

};


class SurfaceUnit
{
public:
    typedef std::shared_ptr<SurfaceUnit> Ptr;
    unsigned long                   id_;                        // ID
    std::vector<LidarPoint::Ptr>    points_;                    // CornerPoints
    std::vector<Edge::Ptr>          edges_;                     // edges
    Vector3d                        centralPos_;                // Position of its central
    std::unordered_map<unsigned long, Frame::Ptr >  observed_frame_; // Frames that contained all 4 points

    Frame::Ptr                      texture_frame_;
    std::vector<Vector2d>           texture_pixel_point_;

    bool isWellDefined_;         // True when the size of points is 3

private:
    static unsigned long factory_id_;    // factory id

public:
    SurfaceUnit();
    SurfaceUnit(unsigned long id);
    SurfaceUnit(
        unsigned long id, 
        const std::vector<LidarPoint::Ptr>& points
    );
    SurfaceUnit(
            unsigned long id,
            LidarPoint::Ptr point,
            Edge::Ptr edge
    );

private:
    void insertObservedFrame ( Frame::Ptr frame );

public:
    bool insertPoint ( LidarPoint::Ptr lp );
    bool insertEdge(Edge::Ptr ep);
    void calcCG(void);
    
    static SurfaceUnit::Ptr createSurfaceUnit();
    static SurfaceUnit::Ptr createSurfaceUnit(
            const std::vector<LidarPoint::Ptr>& points
    );

    static SurfaceUnit::Ptr createSurfaceUnit (
            LidarPoint::Ptr point,
            Edge::Ptr edge
    );

    bool isPointInSU(LidarPoint::Ptr lp);
    bool isEdgeInSU(Edge::Ptr ep);
    bool isFrameInSU(Frame::Ptr fp);

    int addObservedFrames(vector<Frame::Ptr>& frames);
    bool matchTexture();

};


class Surface
{
public:
    typedef std::shared_ptr<Surface> Ptr;
    std::unordered_map<unsigned long, SurfaceUnit::Ptr >    surfaceunits_; // Frames that contained all 4 points
    std::list< Edge::Ptr >                                  half_edges_; // Frames that contained all 4 points
    std::unordered_map<unsigned long, Edge::Ptr >           full_edges_; // Frames that contained all 4 points
    std::unordered_map<unsigned long, LidarPoint::Ptr >     used_points_; // Frames that contained all 4 points
    PointCloud::Ptr pointcloud_;

private:
    static unsigned long factory_id_;    // factory id

public:
    Surface();
    Surface(PointCloud::Ptr pc);

    void setPointCloud(PointCloud::Ptr pc);

    void rebuild();
    void rebuildV2(std::vector<LidarPoint::Ptr>& lps, std::vector<Frame::Ptr>& fps);

    bool insertSurfaceUnit(SurfaceUnit::Ptr supt);
    bool insertHalfEdge(Edge::Ptr ep);
    bool deleteHalfEdge(Edge::Ptr ep);
    bool insertFullEdge(Edge::Ptr ep);
    void setUsedPoint(LidarPoint::Ptr lp);

    bool isInHalfEdgeList(Edge::Ptr ep);
    bool isInFullEdgeList(Edge::Ptr ep);
    bool isInSurfaceUnitList(SurfaceUnit::Ptr supt);
    bool isPointUsed(LidarPoint::Ptr lp);

private:
    double getPointAngleCos(LidarPoint::Ptr base_p, LidarPoint::Ptr p1, LidarPoint::Ptr p2);
    Eigen::Vector2d project2Surface(Eigen::Vector3d pt, Eigen::Isometry3d tf);
    Eigen::Isometry3d calcTransferMatrix(Edge::Ptr ep, SurfaceUnit::Ptr supt);
    LidarPoint::Ptr getNextPoint(Edge::Ptr edge, SurfaceUnit::Ptr su);
    Edge::Ptr getNextEdge(Edge::Ptr edge, SurfaceUnit::Ptr su);
    bool isInSurfaceUnit(Edge::Ptr edge, SurfaceUnit::Ptr su);
};



}

#endif // SURFACE_H
