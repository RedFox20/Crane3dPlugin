// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "Model.h"
#include <vector>
#include <deque>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    struct WayPoint
    {
        double X = 0.0;
        double Y = 0.0;
        double R = 0.0;
        /**
         * Duration in seconds for following this waypoint
         * Duration <= 0.0: no duration, terminate waypoint when we reach it
         */
        double Duration = 0.0;

        // How long to wait after reaching destination?
        double Wait = 0.0;
    };

    /**
     * Simple waypoint based controller of the crane
     */
    class CraneController
    {
        crane3d::Model* Model = nullptr;
        Force Frail, Fcart, Fwind;
        std::deque<WayPoint> WayPoints;

    public:

        CraneController(crane3d::Model* model);

        /** max driving forces applied when following waypoints */
        void SetDrivingForces(Force FrailMax, Force FcartMax, Force FwindMax);

        /**
         * Adds a new waypoint to follow.
         * @note X,Y,R are clamped to Model limits to prevent unreachable coordinates
         * @param X Desired X position
         * @param Y Desired Y position
         * @param R Desired R position
         * @param duration [0.0] Number of seconds to follow+stay at target pos.
         *     If duration <= 0.0: terminate waypoint immediately when we reach it.
         */
        void AddWayPoint(double X, double Y, double R, double duration = 0.0)
        {
            AddWayPoint(WayPoint{ X, Y, R, duration });
        }

        void AddWayPoint(WayPoint p);
        
        /** Set waypoints for the crane to follow */
        void SetWayPoints(const std::vector<WayPoint>& wayPoints);

        /** Clears the current waypoint list */
        void ClearWayPoints();

        /** 
         * Runs the simulation for the specified number of seconds while following waypoints.
         * @param fixedTimeStep Small delta time
         * @param runTimeSeconds Total run time of the controller
         */
        void Run(double fixedTimeStep, double runTimeSeconds);
    };

    //////////////////////////////////////////////////////////////////////
}
