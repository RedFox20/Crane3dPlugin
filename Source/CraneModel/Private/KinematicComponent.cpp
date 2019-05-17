// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "KinematicComponent.h"
#include <cmath>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    void Component::Reset()
    {
        Pos = Vel = 0.0;
        Acc = NetAcc = Accel::Zero();
        Applied = SFriction = KFriction = Fnet = Force::Zero();
    }

    void Component::Update(Accel newAcc, double dt)
    {
        double newPos = integrate_verlet_pos(Pos, Vel, Acc, dt);
        double newVel;
        if (inside_limits(newPos, LimitMin, LimitMax))
        {
            newVel = integrate_verlet_vel(Vel, Acc, newAcc, dt);
        }
        else // pos must be clamped, use Vavg
        {
            newPos = clamp(newPos, LimitMin, LimitMax);
            newVel = average_velocity(Pos, newPos, dt);
        }

        if (VelMax > 0.0 && std::abs(newVel) > VelMax) {
            newVel = sign(newVel) * VelMax;
        }
        Pos = newPos;
        Vel = newVel;
        Acc = newAcc;
    }

    // Simplified Coloumb-Viscous friction model:
    // F = f_c*sign(v) + f_v*v
    void Component::ApplyForce(Force applied)
    {
        applied = ClampForceByPosLimits(applied); // no Fapp at edges
        KFriction = Force{CoeffKineticViscous} * Vel;
        SFriction = Force{CoeffStaticColoumb} * sign(Vel);

        // Coloumb static friction: never greater than net force
        if (SFriction != 0.0 && abs(SFriction) > abs(Fnet))
            SFriction = sign(SFriction) * abs(Fnet);

        Applied = applied;
        Fnet = applied - FrictionDir * (SFriction + KFriction);
        Fnet = dampen(Fnet); // removes sigma sized force flip-flopping
        NetAcc = Fnet / Mass;
    }

    Force Component::ClampForceByPosLimits(Force force) const
    {
        Force F = FrictionDir*force;
        if (F > 0.0 && Pos > (LimitMax-0.01)) return Force::Zero();
        if (F < 0.0 && Pos < (LimitMin+0.01)) return Force::Zero();
        return force;
    }

    //////////////////////////////////////////////////////////////////////
}
