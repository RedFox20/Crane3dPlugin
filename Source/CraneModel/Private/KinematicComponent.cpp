// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "KinematicComponent.h"
#include <cmath>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    constexpr double clamp(double x, double min, double max)
    {
        if (x <= min) return min;
        if (x >= max) return max;
        return x;
    }

    // dampens values that are very close to 0.0
    constexpr double dampen(double x)
    {
        return std::abs(x) < 0.001 ? 0.0 : x;
    }

    //////////////////////////////////////////////////////////////////////

    void Component::Update(Accel new_acc, double dt)
    {
        double newPos = Pos + Vel * dt + Acc.Value*(dt*dt*0.5);
        newPos = clamp(newPos, LimitMin, LimitMax);

        // since position can be clamped, we calculate avg velocity instead
        Vel = average_velocity(Pos, newPos, dt);
        Pos = newPos;
        Acc = new_acc;

        Vel = dampen(Vel); // dampen extremely small velocities
    }

    void Component::ApplyForce(Force applied, Accel g)
    {
        Force Fnormal = Mass * g; // normal force between body and surface
        Friction = Force::Zero;
        if (std::abs(Vel) < 0.001) // static friction
        {
            Force staticMax = CoeffStatic * Fnormal;
            if (abs(applied) > staticMax) // resist up to max static friction
                Friction = sign(applied) * (abs(applied) - staticMax);
            else
                Friction = applied; // cancel out applied force
        }
        else
        {
            Force kinetic = CoeffKinetic * Fnormal;
            Friction = sign(Vel) * kinetic;
        }

        // @note Fnet positive: driving dominates
        // @note Fnet negative: friction dominates
        Applied = applied;
        Fnet = applied - Friction;
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    void Component::ApplyForceNonLinear(Force applied, Accel g, double T, double Ts, double dir)
    {
        Friction = Force{ dir * (Vel*T  +  Ts*sign(Vel)) };
        Applied = applied;
        Fnet = applied - Friction;
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    Force Component::ClampForceByPosLimits(Force force) const
    {
        if (force > 0.0 && Pos >= LimitMax) return Force::Zero;
        if (force < 0.0 && Pos <= LimitMin) return Force::Zero;
        return force;
    }

    //////////////////////////////////////////////////////////////////////
}
