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
        Acc = NetAcc = Accel::Zero;
        Applied = SFriction = KFriction = Fnet = Force::Zero;
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

    // TODO: This is not suitable for crane kinematics model
    void Component::UpdateForceColoumb(Force applied, Accel g)
    {
        SFriction = 0_N;
        KFriction = 0_N;
        if (std::abs(Vel) < 0.001)
        {
            Force Fc = CoeffStatic * (Mass * g);
            Force F = abs(applied) < Fc ? applied : Fc;
            SFriction = sign(applied) * F;
        }
        else
        {
            KFriction = sign(Vel) * CoeffKinetic * (Mass * g);
        }

        Applied = applied;
        Fnet = applied - FrictionDir * (SFriction + KFriction);
        Fnet = dampen(Fnet);
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    // Stribeck friction model:
    // F = (Fc + (Fst - Fc)e^-(|v/vs|)^i)*sign(v) + Kv*v
    // Kv - viscous friction coefficient
    // Simplified pre measured model:
    // F = f_c*sign(v) + f_v*v
    void Component::UpdateForce(Force applied)
    {
        SFriction = Force{CoeffStaticColoumb} * sign(Vel);
        KFriction = Force{CoeffKineticViscous} * Vel;

        Applied = applied;
        Fnet = applied - FrictionDir * (SFriction + KFriction);
        Fnet = dampen(Fnet); // removes sigma sized force flip-flopping
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    Force Component::ClampForceByPosLimits(Force force) const
    {
        Force friction = FrictionDir*force;
        if (friction > 0.0 && Pos > (LimitMax-0.01)) return Force::Zero;
        if (friction < 0.0 && Pos < (LimitMin+0.01)) return Force::Zero;
        return force;
    }

    Accel Component::ClampAccelByPosLimits(Accel accel) const
    {
        Accel friction = FrictionDir*accel;
        if (friction > 0.0 && Pos > (LimitMax-0.01)) return Accel::Zero;
        if (friction < 0.0 && Pos < (LimitMin+0.01)) return Accel::Zero;
        return accel;
    }

    //////////////////////////////////////////////////////////////////////
}
