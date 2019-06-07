// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "Kinematics.h"

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    enum class IntegrationMethod
    {
        // Default; Accurate approximation based on velocity
        VelocityVerlet,
        // classic, very inaccurate: 
        // 1) update pos
        // 2) update vel
        ExplicitEuler,
        // more stable, but still not perfect
        // 1) update vel
        // 2) update pos
        SemiImplicitEuler,
        // Runge-Kutta, 4 samples
        RK4,
    };

    const char* to_string(IntegrationMethod method);

    struct Integration
    {
        double Pos, Vel;
    };

    struct Derivative
    {
        double Dx, Dv; // Dx/dt = velocity, Dv/dt = acceleration
    };

    /**
     * Single force component with its own Position, Velocity, Acceleration and Net Force
     */
    struct Component
    {
        // base properties:
        Mass Mass = 1_kg;  // mass
        double Pos = 0.0;  // current position within limits
        double Vel = 0.0;  // instantaneous velocity
        Accel Acc = 0_ms2; // instantaneous acceleration

        IntegrationMethod Method = IntegrationMethod::VelocityVerlet;

        // limits:
        double LimitMin = 0.0;
        double LimitMax = 0.0;
        double VelMax = 0.0; // velocity limit, 0 = disabled
        double AccMax = 0.0; // acceleration limit, 0 = disabled

        // driving forces:
        Force Applied;   // applied / driving force
        Force SFriction; // static friction
        Force KFriction; // kinematic friction
        Force Fnet;      // net driving force
        Accel NetAcc;    // net driving acceleration

        double FrictionDir = 1.0;

        // Coloumb-Viscous friction model
        // https://www.hindawi.com/journals/mpe/2013/946526/
        double CoeffStaticColoumb  = 5.0;   // resistance to starting movement
        double CoeffKineticViscous = 100.0; // resistance as velocity increases

        Component() = default;
        Component(double pos, double limitMin, double limitMax)
            : Pos{pos}, LimitMin{limitMin}, LimitMax{limitMax} {}

        void SetLimits(double min, double max) { LimitMin = min; LimitMax = max; }

        // Resets all dynamic variables: Pos, Vel, Acc, Fnet, NetAcc
        void Reset();

        // Update pos and vel using "Velocity Verlet" integration
        void Update(Accel newAcc, double dt);

        // Apply driving forces and friction forces
        void ApplyForce(Force applied);

        // Prevent applying force when against frame
        Force ClampForceByPosLimits(Force force) const;

    private:

        Integration IntegratePosVel(Accel newAcc, double dt) const;

        Integration IntegrateVelocityVerlet(Accel newAcc, double dt) const;
        Integration IntegrateExplicitEuler(Accel newAcc, double dt) const;
        Integration IntegrateSemiImplicitEuler(Accel newAcc, double dt) const;
        Integration IntegrateRK4(Accel newAcc, double dt) const;
    };

    //////////////////////////////////////////////////////////////////////
}
