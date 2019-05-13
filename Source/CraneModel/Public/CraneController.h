// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "Model.h"
#include <vector>
#include <deque>

namespace crane3d
{
    struct WayPoint
    {
        double Duration = 1.0; // duration in seconds for following this waypoint
        double X = 0.0;
        double Y = 0.0;
    };

    /**
     * Debug driver controller of the crane
     */
    class CraneController
    {
        crane3d::Model* Model = nullptr;
        Force Frail, Fcart, Fwind;
        std::deque<WayPoint> WayPoints;

    public:
        CraneController(crane3d::Model* model) : Model{model} {}

        // max driving forces applied for following waypoints
        void SetDrivingForces(Force FrailMax, Force FcartMax, Force FwindMax);

        // set waypoints for the crane to follow
        void SetWayPoints(const std::vector<WayPoint>& wayPoints);

        // runs the simulation for the specified number of seconds,
        // while following any waypoints
        void Run(double runTimeSeconds, double fixedTimeStep);

    private:
        
        WayPoint NextWayPoint(double fixedTimeStep);

    };
}