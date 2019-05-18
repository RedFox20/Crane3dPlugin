// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "CraneController.h"
#include <cassert>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    CraneController::CraneController(crane3d::Model* model) : Model{model}
    {
        assert(Model != nullptr && "CraneController Model* cannot be null!");
    }

    void CraneController::SetDrivingForces(Force FrailMax, Force FcartMax, Force FwindMax)
    {
        Frail = FrailMax;
        Fcart = FcartMax;
        Fwind = FwindMax;
    }

    void CraneController::AddWayPoint(WayPoint p)
    {
        p.X = clamp(p.X, Model->Rail.LimitMin, Model->Rail.LimitMax);
        p.Y = clamp(p.Y, Model->Cart.LimitMin, Model->Cart.LimitMax);
        p.R = clamp(p.R, Model->Line.LimitMin, Model->Line.LimitMax);
        WayPoints.push_back(p);
    }

    void CraneController::SetWayPoints(const std::vector<WayPoint>& wayPoints)
    {
        ClearWayPoints();
        for (const WayPoint& p : wayPoints)
            AddWayPoint(p);
    }

    void CraneController::ClearWayPoints()
    {
        WayPoints.clear();
    }

    void CraneController::Run(double fixedTimeStep, double runTimeSeconds)
    {
        double simTime = 0.0;
        for (; simTime < runTimeSeconds; simTime += fixedTimeStep)
        {
            WayPoint wp = WayPoints.empty() ? WayPoint{ 0.0, 0.0, 0.5 } : WayPoints.front();
            double dx = (wp.X - Model->Rail.Pos);
            double dy = (wp.Y - Model->Cart.Pos);
            double dr = (wp.R - Model->Line.Pos);

            if (!WayPoints.empty())
            {
                WayPoint& first = WayPoints.front();
                if (first.Duration > 0.0) // this is a timed waypoint
                {
                    first.Duration -= fixedTimeStep;
                    if (first.Duration < 0.0) {
                        WayPoints.pop_front();
                    }
                }
                else // this is an immediate waypoint, terminate if we arrive closeby
                {
                    if (abs(dx) < 0.001 && abs(dy) < 0.001 && abs(dr) < 0.001) {
                        first.Wait -= fixedTimeStep;
                        if (first.Wait < 0.0) {
                            WayPoints.pop_front();
                        }
                    }
                }
            }

            Force FdriveRail = sign(dx) * Frail;
            Force FdriveCart = sign(dy) * Fcart;
            Force FdriveLine = sign(dr) * Fwind;

            Model->Update(fixedTimeStep, FdriveRail, FdriveCart, FdriveLine);
        }
    }

    //////////////////////////////////////////////////////////////////////
}
