// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "Kinematics.h"

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

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

        // limits:
        double LimitMin = 0.0;
        double LimitMax = 0.0;
        double VelMax = 0.0; // velocity limit, 0 = disabled
        double AccMax = 0.0; // acceleration limit, 0 = disabled

        // driving forces:
        Force Applied;   // applied / driving force
        Force SFriction; // static friction
        Force KFriction; // kinematic friction
        Force Fnet;      // net force
        Accel NetAcc;    // net driving acceleration

        double FrictionDir = 1.0;

        // Coloumb-Viscous friction model
        // https://www.hindawi.com/journals/mpe/2013/946526/
        double CoeffStaticColoumb  = 5.0; // resistance to starting movement
        double CoeffKineticViscous = 100.0; // resistance as velocity increases

        // friction coefficient for Steel-Steel (depends highly on type of steel)
        // https://hypertextbook.com/facts/2005/steel.shtml
        double CoeffStatic = 0.8; // static coeff, dry surface
        double CoeffKinetic = 0.7; // kinetic coeff, dry surface

        Component() = default;
        Component(double pos, double limitMin, double limitMax)
            : Pos{pos}, LimitMin{limitMin}, LimitMax{limitMax} {}

        void SetLimits(double min, double max) { LimitMin = min; LimitMax = max; }

        // Resets all dynamic variables: Pos, Vel, Acc, Fnet, NetAcc
        void Reset();

        // Update pos and vel using "Velocity Verlet" integration
        void Update(Accel new_acc, double dt);

        // Apply driving forces
        void UpdateForceColoumb(Force applied, Accel g);

        // Apply driving forces and Stribeck friction model
        void UpdateForce(Force applied);

        // Prevent applying force when against frame
        Force ClampForceByPosLimits(Force force) const;

        // Prevent applying acceleration when against frame
        Accel ClampAccelByPosLimits(Accel accel) const;
    };

    //////////////////////////////////////////////////////////////////////
}
