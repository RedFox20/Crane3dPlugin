// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "KinematicComponent.h"
#include <cmath>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    const char* to_string(IntegrationMethod method)
    {
        switch (method)
        {
            case IntegrationMethod::VelocityVerlet: return "VelocityVerlet";
            case IntegrationMethod::ExplicitEuler: return "ExplicitEuler";
            case IntegrationMethod::SemiImplicitEuler: return "SemiImplicitEuler";
            case IntegrationMethod::RK4: return "RK4";
            default: return "<IntegrationMethod>";
        }
    }

    void Component::Reset()
    {
        Pos = Vel = 0.0;
        Acc = NetAcc = Accel::Zero();
        Applied = SFriction = KFriction = Fnet = Force::Zero();
    }

    void Component::Update(Accel newAcc, double dt)
    {
        Integration r = IntegratePosVel(newAcc, dt);

        // pos must be clamped, use Vavg
        if (!inside_limits(r.Pos, LimitMin, LimitMax)) {
            r.Pos = clamp(r.Pos, LimitMin, LimitMax);
            r.Vel = average_velocity(Pos, r.Pos, dt);
        }

        if (VelMax > 0.0 && std::abs(r.Vel) > VelMax) {
            r.Vel = sign(r.Vel) * VelMax;
        }
        Pos = r.Pos;
        Vel = r.Vel;
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

    Integration Component::IntegratePosVel(Accel newAcc, double dt) const
    {
        switch (Method)
        {
        default:
        case IntegrationMethod::VelocityVerlet:    return IntegrateVelocityVerlet(newAcc, dt);
        case IntegrationMethod::ExplicitEuler:     return IntegrateExplicitEuler(newAcc, dt);
        case IntegrationMethod::SemiImplicitEuler: return IntegrateSemiImplicitEuler(newAcc, dt);
        case IntegrationMethod::RK4:               return IntegrateRK4(newAcc, dt);
        }
    }

    Integration Component::IntegrateVelocityVerlet(Accel newAcc, double dt) const
    {
        Integration r {};
        r.Pos = integrate_verlet_pos(Pos, Vel, Acc, dt);
        r.Vel = integrate_verlet_vel(Vel, Acc, newAcc, dt);
        return r;
    }

    Integration Component::IntegrateExplicitEuler(Accel newAcc, double dt) const
    {
        Integration r {};
        r.Pos = integrate_euler_pos(Pos, Vel, dt); // 1 step behind
        r.Vel = integrate_euler_vel(Vel, newAcc, dt);
        return r;
    }

    Integration Component::IntegrateSemiImplicitEuler(Accel newAcc, double dt) const
    {
        Integration r {};
        r.Vel = integrate_euler_vel(Vel, newAcc, dt); // vel updated first
        r.Pos = integrate_euler_pos(Pos, r.Vel, dt); // and use new vel
        return r;
    }

    inline Derivative RK4_derive(double initialPos, double initialVel, 
                                 Accel newAcc, double dt, const Derivative& d)
    {
        double pos2 = initialPos + d.Dx * dt;
        double vel2 = initialVel + d.Dv * dt;
        // @todo This needs more work
        double acc2 = newAcc.Value;
        return { vel2, acc2 };
    }

    Integration Component::IntegrateRK4(Accel newAcc, double dt) const
    {
        Derivative k1 = RK4_derive(Pos, Vel, newAcc, 0.0, {0.0, 0.0});
        Derivative k2 = RK4_derive(Pos, Vel, newAcc, dt*0.5, k1);
        Derivative k3 = RK4_derive(Pos, Vel, newAcc, dt*0.5, k2);
        Derivative k4 = RK4_derive(Pos, Vel, newAcc, dt,     k3);

        double dxdt = (1.0 / 6.0) * (k1.Dx + 2.0*(k2.Dx + k3.Dx) + k4.Dx);
        double dvdt = (1.0 / 6.0) * (k1.Dv + 2.0*(k2.Dv + k3.Dv) + k4.Dv);
        Integration r {};
        r.Pos = Pos + dxdt * dt;
        r.Vel = Vel + dvdt * dt;
        return r;
    }

    //////////////////////////////////////////////////////////////////////
}
