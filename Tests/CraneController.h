// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include <Model.h>
#include <vector>
#include <deque>
#include <cassert>

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
        void SetDrivingForces(Force FrailMax, Force FcartMax, Force FwindMax)
        {
            Frail = FrailMax;
            Fcart = FcartMax;
            Fwind = FwindMax;
        }

        // set waypoints for the crane to follow
        void SetWayPoints(const std::vector<WayPoint>& wayPoints)
        {
            WayPoints.assign(wayPoints.begin(), wayPoints.end());
        }

        // runs the simulation for the specified number of seconds,
        // while following any waypoints
        void Run(double runTimeSeconds, double fixedTimeStep)
        {
            assert(Model != nullptr);
            Force FdriveRail, FdriveCart, FdriveWind;
            double simulationTime = 0.0;

            printf("Controller Run: T:%.1fs dt:%.3fs wp:%llu\n", runTimeSeconds, fixedTimeStep, WayPoints.size());

            while (simulationTime < runTimeSeconds)
            {
                WayPoint wp = NextWayPoint(fixedTimeStep);

                double dx = (wp.X - Model->Rail.Pos);
                double dy = (wp.Y - Model->Cart.Pos);

                Force FdriveRail = sign(dx) * Frail;
                Force FdriveCart = sign(dy) * Fcart;
                Force FdriveLine = 0_N;

                Model->Update(fixedTimeStep, FdriveRail, FdriveCart, FdriveLine);
                simulationTime += fixedTimeStep;
            }
        }

    private:
        
        WayPoint NextWayPoint(double fixedTimeStep)
        {
            WayPoint pt { 0.0, 0.0, 0.0 };
            if (!WayPoints.empty())
            {
                WayPoint& first = WayPoints.front();
                pt = first;
                first.Duration -= fixedTimeStep;
                if (first.Duration < 0.0) {
                    printf("pop waypoint %.2f, %.2f\n", first.X, first.Y);
                    WayPoints.pop_front();
                }
            }
            return pt;
        }

    };
}