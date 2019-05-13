// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "CraneController.h"
#include <cassert>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    void CraneController::SetDrivingForces(Force FrailMax, Force FcartMax, Force FwindMax)
    {
        Frail = FrailMax;
        Fcart = FcartMax;
        Fwind = FwindMax;
    }

    void CraneController::SetWayPoints(const std::vector<WayPoint>& wayPoints)
    {
        WayPoints.assign(wayPoints.begin(), wayPoints.end());
    }

    void CraneController::Run(double runTimeSeconds, double fixedTimeStep)
    {
        assert(Model != nullptr);
        double simulationTime = 0.0;

        printf("Controller Run: T:%.1fs dt:%.3fs wp:%zu\n", runTimeSeconds, fixedTimeStep, WayPoints.size());

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

    WayPoint CraneController::NextWayPoint(double fixedTimeStep)
    {
        WayPoint pt{ 0.0, 0.0, 0.0 };
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

    //////////////////////////////////////////////////////////////////////
}
