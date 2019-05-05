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
    static double dampen(double x)
    {
        return std::abs(x) < 0.001 ? 0.0 : x;
    }
    static Force dampen(Force force)
    {
        return { std::abs(force.Value) < 0.001 ? 0.0 : force.Value };
    }

    //////////////////////////////////////////////////////////////////////

    void Component::Reset()
    {
        Pos = Vel = 0.0;
        Acc = NetAcc = Accel::Zero;
        Applied = SFriction = KFriction = Fnet = Force::Zero;
    }

    void Component::Update(Accel new_acc, double dt)
    {
        if (!Const)
        {
            double newPos = Pos + Vel * dt + Acc.Value*(dt*dt*0.5);
            newPos = clamp(newPos, LimitMin, LimitMax);

            // since position can be clamped, we calculate avg velocity instead
            Vel = average_velocity(Pos, newPos, dt);
            Pos = newPos;

            if (VelMax > 0.0 && std::abs(Vel) > VelMax) {
                Vel = sign(Vel) * VelMax;
            }
            if (AccMax > 0.0 && abs(Acc) > AccMax) {
                Acc = Accel{sign(Acc) * AccMax};
            }
        }
        Acc = new_acc;
        //Vel = dampen(Vel); // dampen extremely small velocities
    }

    void Component::ApplyForce(Force applied, Accel g)
    {
        Force Fnormal = Mass * g; // normal force between body and surface
        SFriction = Force::Zero;
        KFriction = Force::Zero;
        if (std::abs(Vel) < 0.001) // static friction
        {
            Force staticMax = CoeffStatic * Fnormal;
            if (abs(applied) > staticMax) // resist up to max static friction
                SFriction = sign(applied) * (abs(applied) - staticMax);
            else
                SFriction = applied; // cancel out applied force
        }
        else
        {
            KFriction = sign(Vel) * CoeffKinetic * Fnormal;
        }

        // @note Fnet positive: driving dominates
        // @note Fnet negative: friction dominates
        Applied = applied;
        Fnet = applied - FrictionDir * (SFriction + KFriction);
        Fnet = dampen(Fnet);
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    void Component::ApplyForceNonLinear(Force applied, Accel g, double T, double Ts)
    {
        SFriction = Force{ Vel*T };
        KFriction = Force{ Ts*sign(Vel) };
        Applied = applied;
        Fnet = applied - FrictionDir * (SFriction + KFriction);
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    Force Component::ClampForceByPosLimits(Force force) const
    {
        if (FrictionDir >= 0.0)
        {
            if (force > 0.0 && Pos >= LimitMax) return Force::Zero;
            if (force < 0.0 && Pos <= LimitMin) return Force::Zero;
        }
        else
        {
            if (force < 0.0 && Pos >= LimitMax) return Force::Zero;
            if (force > 0.0 && Pos <= LimitMin) return Force::Zero;
        }
        return force;
    }

    //////////////////////////////////////////////////////////////////////
}
