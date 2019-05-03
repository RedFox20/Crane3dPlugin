// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#include "Model.h"
#include <cmath>
#include <cstdio>
#include <sstream> // std::stringstream
#include <iomanip> // std::setprecision

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    // time between each discrete step of the simulation
    static constexpr double _PI = 3.14159265358979323846;
    static constexpr double _90degs = (_PI / 2);

    const Force Force::Zero { 0.0 };
    const Mass  Mass::Zero  { 0.0 };
    const Accel Accel::Zero { 0.0 };

    constexpr double clamp(double x, double min, double max)
    {
        if (x <= min) return min;
        if (x >= max) return max;
        return x;
    }

    //////////////////////////////////////////////////////////////////////
    
    Model::Model()
    {
        // Invert the alfa angle as required by the model
        // α := 90° - α
        Alfa = _90degs - Alfa;
        Δα = 0.0;
    }

    ModelState Model::GetState() const
    {
        // Restore the internal alfa representation
        ModelState s;
        s.RailOffset = X;
        s.CartOffset = Y;
        s.LiftLine   = R;

        // Formulas given by 3DCrane mathematical model description
        if (Type == ModelType::Linear)
        {
            s.Alfa = Δα;
            s.Beta = Δβ;
            s.PayloadX = X + R * Δβ;
            s.PayloadY = Y - R * Δα;
            s.PayloadZ = -R;
        }
        else
        {
            // Calculate payload position from Xw,Yw,α,β,R
            double α = _90degs - Alfa;
            s.Alfa = α;
            s.Beta = Beta;
            s.PayloadX = X + R * sin(α) * sin(Beta);
            s.PayloadY = Y + R * cos(α);
            s.PayloadZ = -R * sin(α) * cos(Beta);
        }
        return s;
    }

    const char* ToString(ModelType type)
    {
        switch (type)
        {
            default:
            case ModelType::Linear:             return "Linear";
            case ModelType::NonLinearConstLine: return "NonLinearConstLine";
            case ModelType::NonLinearComplete:  return "NonLinearComplete";
            case ModelType::NonLinearOriginal:  return "NonLinearOriginal";
        }
    }

    std::wstring Model::GetStateDebugText() const
    {
        ModelState state = GetState();
        std::wstringstream ss;
        ss << L"Model: " << ToString(Type) << L'\n';
        ss << std::setprecision(2);
        ss << L"   X " << X     << L'\n';
        ss << L"   Y " << Y     << L'\n';
        ss << L"   R " << R     << L'\n';
        ss << L"  vX " << X_vel << L" m/s \n";
        ss << L"  vY " << Y_vel << L" m/s \n";
        ss << L"  vR " << R_vel << L" m/s \n";
        ss << L"  pX " << state.PayloadX << L'\n';
        ss << L"  pY " << state.PayloadY << L"\n";
        ss << L"  pZ " << state.PayloadZ << L"\n";
        if (Type == ModelType::Linear)
        {
            ss << L"  Δα " << Δα     << L'\n';
            ss << L"  Δβ " << Δβ     << L'\n';
            ss << L" vΔα " << Δα_vel << L" u/s \n";
            ss << L" vΔβ " << Δβ_vel << L" u/s \n";
        }
        else
        {
            ss << L"  α " << Alfa     << L'\n';
            ss << L"  β " << Beta     << L'\n';
            ss << L" vα " << Alfa_vel << L" m/s \n";
            ss << L" vβ " << Beta_vel << L" m/s \n";
        }
        ss << L" ΣaCart " << ANetcart.Value << L" m/s² \n";
        ss << L" ΣaRail " << ANetrail.Value << L" m/s² \n";
        ss << L" ΣaWind " << ANetwind.Value << L" m/s² \n";
        return ss.str();
    }

    void ModelState::Print() const
    {
        printf("Alfa: %.2f Beta: %.2f Rail: %.2f Cart: %.2f Line: %.2f "
               "X: %.2f Y: %.2f Z: %.2f\n",
                Alfa, Beta, RailOffset, CartOffset, LiftLine,
                PayloadX, PayloadY, PayloadZ);
    }

    //////////////////////////////////////////////////////////////////////

    ModelState Model::UpdateFixed(double fixedTime, double deltaTime, Force Frail, Force Fcart, Force Fwind)
    {
        // we run the simulation with a constant time step
        SimulationTime += deltaTime;
        int iterations = static_cast<int>(SimulationTime / fixedTime);
        SimulationTime -= floor(SimulationTime);

        for (int i = 0; i < iterations; ++i)
        {
            Update(fixedTime, Frail, Fcart, Fwind);
        }

        // and finally, return the observable current state
        return GetState();
    }

    ModelState Model::Update(double deltaTime, Force Frail, Force Fcart, Force Fwind)
    {
        switch (Type)
        {
            default:
            case ModelType::Linear: BasicLinearModel(deltaTime, Frail, Fcart, Fwind); break;
            case ModelType::NonLinearConstLine: NonLinearConstLine(deltaTime, Frail, Fcart, Fwind); break;
            case ModelType::NonLinearComplete: NonLinearCompleteModel(deltaTime, Frail, Fcart, Fwind); break;
            case ModelType::NonLinearOriginal: NonLinearOriginalModel(deltaTime, Frail, Fcart, Fwind); break;
        }

        GetState().Print();

        // and finally, return the observable current state
        return GetState();
    }

    //////////////////////////////////////////////////////////////////////

    // `drivingAccel` must overcome `frictionAccel` in order to have an effect.
    double ApplyStaticFriction(double drivingAccel, double frictionAccel)
    {
        if (std::abs(drivingAccel) <= std::abs(frictionAccel))
            return 0.0;
        return sign(drivingAccel) * (std::abs(drivingAccel) - std::abs(frictionAccel));
    }

    Force Model::KineticFriction(double velocity, Mass m, double μKinetic) const
    {
        double absVel = std::abs(velocity);
        if (absVel >= 0.001) // moving
        {
            Force Fnormal = m*g; // normal force between body and surface
            Force Fkinetic = μKinetic * Fnormal;
            //if (absVel > Fkinetic)
                return sign(velocity) * Fkinetic;
            //else // vel <= Fk
            //    return Force{ velocity };
        }
        return Force::Zero;
    }

    Force Model::StaticFriction(Force Fapplied, double velocity, Mass m, double μStatic) const
    {
        if (std::abs(velocity) < 0.001) // still
        {
            Force Fnormal = m*g; // normal force between body and surface
            Force FstaticMax = μStatic * Fnormal;
            // resist force up to max static friction force
            if (abs(Fapplied) > FstaticMax)
            {
                return sign(Fapplied) * (abs(Fapplied) - FstaticMax);
            }
        }
        return Force::Zero;
    }

    Force Model::NetForce(Force Fapplied, double velocity, Mass m, double μStatic, double μKinetic) const
    {
        Force Fstatic  = StaticFriction(Fapplied, velocity, m, μStatic);
        Force Fkinetic = KineticFriction(velocity, m, μKinetic);
        Force Fnet = Fapplied - (Fstatic + Fkinetic);
        // @note Fnet positive: driving dominates
        // @note Fnet negative: friction dominates
        return Fnet;
    }

    void Model::PrepareBasicRelations(Force Frail, Force Fcart, Force Fwind)
    {
        // prepare basic relations
        // we recalculate every frame to allow dynamically changing the model parameters
        u1 = Fcart.Value / Mcart.Value;              // u1 = Fy / Mw        | cart accel
        u2 = Frail.Value / (Mcart + Mpayload).Value; // u2 = Fx / (Mw + Mc) | rail accel
        u3 = Fwind.Value / Mpayload.Value;           // u3 = Fr / Mc        | line accel

        T1 = CartFriction / Mcart.Value;              // T1 = Ty / Mw        | cart friction accel
        T2 = RailFriction / (Mcart + Mpayload).Value; // T2 = Tx / (Mw + Mc) | rail friction accel
        T3 = WindingFriction / Mpayload.Value;        // T3 = Tr / Mc        | line winding friction accel

        N1 = ApplyStaticFriction(u1, T1); // cart net accel
        N2 = ApplyStaticFriction(u2, T2); // rail net accel
        N3 = ApplyStaticFriction(u3, T3); // line net accel
                
        ADrcart = Fcart.Value / Mcart.Value;              // u1 = Fy / Mw        | cart driving accel
        ADrrail = Frail.Value / (Mcart + Mpayload).Value; // u2 = Fx / (Mw + Mc) | rail driving accel
        ADrwind = Fwind.Value / Mpayload.Value;           // u3 = Fr / Mc        | wind driving accel

        AFrcart = CartFriction / Mcart.Value;              // T1 = Ty / Mw        | cart friction accel
        AFrrail = RailFriction / (Mcart + Mpayload).Value; // T2 = Tx / (Mw + Mc) | rail friction accel
        AFrwind = WindingFriction / Mpayload.Value;        // T3 = Tr / Mc        | line winding friction accel

        // these are cable driven friction coefficients:
        μ1 = Mpayload / Mcart;           // μ1 = Mc / Mw
        μ2 = Mpayload / (Mcart + Mrail); // μ2 = Mc / (Mw + Ms)

        // New friction model
        Mass Mcartpayload = Mcart+Mpayload;
        Mass Mall = Mrail+Mcart+Mpayload;
        Force FnetCart = NetForce(Fcart, Y_vel, Mcartpayload, μStaticDrySteel, μKineticDrySteel);
        Force FnetRail = NetForce(Frail, X_vel, Mall,         μStaticDrySteel, μKineticDrySteel);
        Force FnetWind = NetForce(Fwind, R_vel, Mpayload,     μStaticDrySteel, μKineticDrySteel);

        ANetcart = FnetCart / Mcartpayload;
        ANetrail = FnetRail / Mall;
        ANetwind = FnetWind / Mpayload;
    }

    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        // calculate force driven accelerations
        Accel aX  = ANetrail + μ2 * Δβ * ANetwind;
        Accel aY  = ANetcart - μ1 * Δα * ANetwind;
        Accel aΔα =  (aY - g*Δα - 2*Δα_vel*R_vel) / R;
        Accel aΔβ = -(aX + g*Δβ + 2*Δβ_vel*R_vel) / R;
        Accel aR  = g - ANetwind;
        
        // integrate force driven velocities
        X_vel  = integrate_velocity(X_vel,  aX,  dt);
        Y_vel  = integrate_velocity(Y_vel,  aY,  dt);
        Δα_vel = integrate_velocity(Δα_vel, aΔα, dt);
        Δβ_vel = integrate_velocity(Δβ_vel, aΔβ, dt);
        R_vel  = integrate_velocity(R_vel,  aR,  dt);

        // derive new positions within limits
        double X2  = clamp(X  + X_vel*dt,  RailLimitMin, RailLimitMax);
        double Y2  = clamp(Y  + Y_vel*dt,  CartLimitMin, CartLimitMax);
        double R2  = clamp(R  + R_vel*dt,  LineLimitMin, LineLimitMax);
        double Δα2 = clamp(Δα + Δα_vel*dt, -0.2, +0.2);
        double Δβ2 = clamp(Δβ + Δα_vel*dt, -0.2, +0.2);

        // calculate ACTUAL average velocities (due to limits)
        X_vel  = average_velocity(X, X2, dt);
        Y_vel  = average_velocity(Y, Y2, dt);
        R_vel  = average_velocity(R, R2, dt);
        Δα_vel = average_velocity(Δα, Δα2, dt);
        Δβ_vel = average_velocity(Δβ, Δβ2, dt);

        // update positions for next frame
        X = X2;
        Y = Y2;
        //R = R2; // @todo keep this const in a better way
        Δα = Δα2;
        Δβ = Δβ2;
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearConstLine(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double sA = sin(Alfa); double sB = sin(Beta);
        double cA = cos(Alfa); double cB = cos(Beta);
        double sA2 = sA*sA; double sB2 = sB*sB; double cA2 = cA*cA;

        double A = 1 + μ1 * cA2 + μ2 * sA2*sB2;
        double B = 1 + μ1;
        double V1 = μ1*R*cA*(Alfa_vel*Alfa_vel + sA2*Beta_vel*Beta_vel) + μ1*G*cA*sA*cB;
        double V2 = μ2*R*sB*(sA*sA2*Beta_vel*Beta_vel + sA*Alfa_vel*Alfa_vel) + μ2*G*sA2*cB*sB;
        double V3 = B*G*cA*cB + B*R*cA*sA*Beta_vel*Beta_vel + (μ1 - μ2*sB2)*R*cA*sA*Alfa_vel*Alfa_vel;
        double V4 = μ2*R*sA*cB*sB*(Alfa_vel*Alfa_vel + sA2*Beta_vel*Beta_vel) + (B + (μ2 - μ1)*sA2)*G*sB + 2*R*A*cA*Alfa_vel*Beta_vel;

        double aY = ((1 + μ2*sA2*sB2)*N1 - μ1*cA*sA*sB*N2 + V1) / A;
        double aX = (-μ2*cA*sA*sB*N1 + (1 + μ1*cA2)*N2 + V2) / A;
        double aAlfa = ((1 + μ2*sB2)*sA*N1 - B*cA*sB*N2 + V3) / (R*A);
        double aBeta = (μ2*cA*sA*cB*sB*N1 - (B - μ1 * sA2)*cB*N2 - V4) / (R*A*sA);

        Y += dt * Y_vel;
        X += dt * X_vel;
        Alfa += dt * Alfa_vel;
        Beta += dt * Beta_vel;

        Y_vel += dt * aY;
        X_vel += dt * aX;
        Alfa_vel += dt * aAlfa;
        Beta_vel += dt * aBeta;

        ApplyLimits();
    }
    
    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearCompleteModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        double sA = sin(Alfa);
        double sB = sin(Beta);
        double cA = cos(Alfa);
        double cB = cos(Beta);
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double V5 = cA * sA*Beta_vel*Beta_vel*R - 2*R_vel*Alfa_vel + G * cA*cB;
        double V6 = 2 * Beta_vel*(cA*Alfa_vel*R + sA * R_vel) + G * sB;
        double V7 = sA * sA*Beta_vel*Beta_vel*R + G * sA*cB + Alfa_vel * Alfa_vel*R;

        double aY = N1 + μ1 * cA * N3;
        double aX = N2 + μ2 * sA * sB * N3;
        double aAlfa = (sA * N1 - cA * sB * N2 + (μ1 - μ2 * sB*sB) * cA*sA*N3 * V5) / R;
        double aBeta = -(cB*N2 + μ2 * sA*cB*sB*N3 + V6) / (sA*R);
        double aR = -cA * N1 - sA * sB*N2 - (1 + μ1 * cA*cA + μ2 * sA*sA*sB*sB)*N3 + V7;

        Y += dt * Y_vel;
        X += dt * X_vel;
        Alfa += dt * Alfa_vel;
        Beta += dt * Beta_vel;
        R += dt * R_vel;

        Y_vel += dt * aY;
        X_vel += dt * aX;
        Alfa_vel += dt * aAlfa;
        Beta_vel += dt * aBeta;
        R_vel += dt * aR;

        ApplyLimits();
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearOriginalModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double sA = sin(Alfa); double sB = sin(Beta);
        double cA = cos(Alfa); double cB = cos(Beta);
        double sA2 = sA*sA; double sB2 = sB*sB;
        double μ1cA = μ1*cA;
        double μ2sAsB = μ2*sA*sB;
        double βv2 = Beta_vel*Beta_vel;

        double V5 = cA*sA*βv2*R - 2 * R_vel*Alfa_vel + G * cA*cB;
        double V6 = 2 * Beta_vel*(cA*Alfa_vel*R + sA * R_vel) + G * sB;
        double V7 = sA2*βv2*R + G * sA*cB + Alfa_vel * Alfa_vel*R;

        double Tsx = 5 / (Mcart + Mrail).Value; // 1.490
        double Tsy = 7.5 / Mcart.Value; // 6.493
        double Tsz = 10 / Mpayload.Value; // 10

        double aWind = -(R_vel * AFrwind + Tsz * sign(R_vel)); // NET winding accel
        double cartFr = (Y_vel * AFrcart + Tsy * sign(Y_vel)); // some sort of cart friction accel
        double railFr = (X_vel * AFrrail + Tsx * sign(X_vel)); // some sort of rail friction accel

        double aY = ADrcart - cartFr + μ1cA*u3 - μ1cA*aWind;
        double aX = ADrrail - railFr + μ2sAsB*u3 - μ2sAsB*aWind;
        double aAlfa = (-cartFr*sA
            + ADrcart * sA + cA*R * sA*βv2 - cA*ADrrail * sB + cA*cB*G
            - cA*μ2sAsB*sB*u3 + cA*μ2sAsB*sB*aWind + cA*railFr*sB
            + μ1cA*u3 * sA - μ1cA*aWind*sA - 2 * R_vel * Alfa_vel
            ) / R;
        double aBeta = -(
            cB*ADrrail + G * sB + 2 * R * cA*Alfa_vel * Beta_vel
            + cB*μ2sAsB*u3 - cB*μ2sAsB*aWind
            + 2 * R_vel * sA*Beta_vel - cB*railFr
            ) / (R * sA);
        double aR = cA*cartFr - cA*ADrcart
            + R * sA2*βv2 - ADrrail * sA*sB
            + sA*cB*G
            - μ2sAsB*sA*sB*u3 + μ2sAsB*sA*sB*aWind
            + railFr*sA*sB
            - μ1*u3 + μ1*u3*sA2
            + μ1 * aWind - μ1 * aWind*sA2
            + R * Alfa_vel * Alfa_vel - u3 + aWind;

        Y += dt * Y_vel;
        X += dt * X_vel;
        Alfa += dt * Alfa_vel;
        Beta += dt * Beta_vel;
        R += dt * R_vel;

        Y_vel += dt * aY;
        X_vel += dt * aX;
        Alfa_vel += dt * aAlfa;
        Beta_vel += dt * aBeta;
        R_vel += dt * aR;

        ApplyLimits();
    }

    //////////////////////////////////////////////////////////////////////

    // dampen values that are very close to 0.0
    constexpr double Dampen(double x)
    {
        return std::abs(x) < 0.0000001 ? 0.0 : x;
    }

    void Model::ApplyLimits()
    {
        X = clamp(X, RailLimitMin, RailLimitMax);
        Y = clamp(Y, CartLimitMin, CartLimitMax);
        R  = clamp(R, LineLimitMin, LineLimitMax);
        Alfa = clamp(Alfa, -_PI, +_PI);
        Beta = clamp(Beta, -_PI, +_PI);

        // Δα and Δβ must be very small by definition
        Δα = clamp(Δα, -0.2, +0.2);
        Δβ = clamp(Δβ, -0.2, +0.2);
    }

    void Model::DampenAllValues()
    {
        Y   = Dampen(Y);
        X   = Dampen(X);
        Alfa = Dampen(Alfa);
        Beta = Dampen(Beta);
        R    = Dampen(R);

        Y_vel   = Dampen(Y_vel);
        X_vel   = Dampen(X_vel);
        Alfa_vel = Dampen(Alfa_vel);
        Beta_vel = Dampen(Beta_vel);
        R_vel    = Dampen(R_vel);
    }

    //////////////////////////////////////////////////////////////////////
}
