// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#include "Model.h"
#include <cmath>
#include <cstdio>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    // time between each discrete step of the simulation
    static constexpr double SimulationStep = 0.001;
    static constexpr double _PI = 3.14159265358979323846;
    static constexpr double _90degs = (_PI / 2);

    constexpr double sign(double x)
    {
        return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0);
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
        else if (Type == ModelType::Linear2)
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

    void ModelState::Print() const
    {
        printf("Alfa: %.2f Beta: %.2f Rail: %.2f Cart: %.2f Line: %.2f "
               "X: %.2f Y: %.2f Z: %.2f\n",
                Alfa, Beta, RailOffset, CartOffset, LiftLine,
                PayloadX, PayloadY, PayloadZ);
    }

    //////////////////////////////////////////////////////////////////////

    ModelState Model::Update(double deltaTime, double Frail, double Fcart, double Fline)
    {
        // we run the simulation with a constant time step
        SimulationTime += deltaTime;
        int iterations = static_cast<int>(SimulationTime / SimulationStep);
        SimulationTime -= floor(SimulationTime);

        for (int i = 0; i < iterations; ++i)
        {
            switch (Type)
            {
                case ModelType::Linear:
                    BasicLinearModel(SimulationStep, Frail, Fcart);
                    break;
                case ModelType::Linear2:
                    BasicLinearModel2(SimulationStep, Frail, Fcart);
                    break;
                case ModelType::NonLinearConstantLine:
                    NonLinearConstantPendulum(SimulationStep, Frail, Fcart);
                    break;
                case ModelType::NonLinearComplete:
                    NonLinearCompleteModel(SimulationStep, Frail, Fcart, Fline);
                    break;
                case ModelType::NonLinearOriginal:
                    NonLinearOriginalModel(SimulationStep, Frail, Fcart, Fline);
                    break;
            }

            ApplyLimits();
            GetState().Print();
        }

        // and finally, return the observable current state
        return GetState();
    }

    //////////////////////////////////////////////////////////////////////

    // `drivingAccel` must overcome `frictionAccel` in order to have an effect.
    double ApplyStaticFriction(double drivingAccel, double frictionAccel)
    {
        if (abs(drivingAccel) <= abs(frictionAccel))
            return 0.0;
        return sign(drivingAccel) * (abs(drivingAccel) - abs(frictionAccel));
    }

    /**
     * Calculates net acceleration from multiple acceleration and velocity forces
     * @param inputAccel Accel. force put into the system
     * @param frictionAccel Counteracting MAX static friction accel.
     * @param currentVel Current velocity
     */
    double GetNetAccel(double inputAccel, double frictionAccel, double currentVel)
    {
        return inputAccel - currentVel * frictionAccel - (frictionAccel*0.1)*sign(currentVel);
    }

    void Model::PrepareBasicRelations(double Frail, double Fcart, double Fline)
    {
        // prepare basic relations
        // we recalculate every frame to allow dynamically changing the model parameters
        u1 = Fcart / Mcart;              // u1 = Fy / Mw        | cart acceleration force
        u2 = Frail / (Mcart + Mpayload); // u2 = Fx / (Mw + Mc) | rail acceleration force
        u3 = Fline / Mpayload;           // u3 = Fr / Mc        | line acceleration force

        T1 = CartFriction / Mcart;              // T1 = Ty / Mw        | cart friction acceleration force
        T2 = RailFriction / (Mcart + Mpayload); // T2 = Tx / (Mw + Mc) | rail friction acceleration force
        T3 = LineFriction / Mpayload;           // T3 = Tr / Mc        | line friction acceleration force
        
        // friction coefficients:
        μ1 = Mpayload / Mcart;           // μ1 = Mc / Mw
        μ2 = Mpayload / (Mcart + Mrail); // μ2 = Mc / (Mw + Ms)

        N1 = ApplyStaticFriction(u1, T1); // cart net acceleration force
        N2 = ApplyStaticFriction(u2, T2); // rail net acceleration force
        N3 = ApplyStaticFriction(u3, T3); // line net acceleration force

        ADrcart = Fcart / Mcart;              // u1 = Fy / Mw        | cart driving accel
        ADrrail = Frail / (Mcart + Mpayload); // u2 = Fx / (Mw + Mc) | rail driving accel
        ADrwind = Fline / Mpayload;           // u3 = Fr / Mc        | wind driving accel

        AFrcart = CartFriction / Mcart;              // T1 = Ty / Mw        | cart friction accel
        AFrrail = RailFriction / (Mcart + Mpayload); // T2 = Tx / (Mw + Mc) | rail friction accel
        AFrwind = LineFriction / Mpayload;           // T3 = Tr / Mc        | line friction accel

        ANetcart = GetNetAccel(ADrcart, AFrcart, X_vel);
        ANetrail = GetNetAccel(ADrrail, AFrrail, Y_vel);
        ANetwind = GetNetAccel(ADrwind, AFrwind, R_vel);
    }

    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, double Frail, double Fcart)
    {
        PrepareBasicRelations(Frail, Fcart, 0.0);

        // calculate accelerations:
        double aY = ANetcart - μ1 * Δα * ANetwind;
        double aX = ANetrail + μ2 * Δβ * ANetwind;
        double aΔα = +(aY - G*Δα - 2*Δα_vel*R_vel) / R;
        double aΔβ = -(aX + G*Δβ + 2*Δβ_vel*R_vel) / R;
        double aR  = -ANetwind + G;
        
        // OUTPUT: apply current velocity
        Y += dt * Y_vel;
        X += dt * X_vel;
        //R  += dt * R_vel;
        Δα += dt * Δα_vel;
        Δβ += dt * Δβ_vel;

        // compute next velocities  v = v0 + a * Δt
        Y_vel += dt * aY;
        X_vel += dt * aX;
        Δα_vel += dt * aΔα;
        Δβ_vel += dt * aΔβ;
        R_vel  += dt * aR;
    }
    
    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, double Frail, double Fcart)
    {
        PrepareBasicRelations(Frail, Fcart, 0.0);

        // calculate accelerations:
        double aY = ANetcart - μ1 * Δα * ANetwind;
        double aX = ANetrail + μ2 * Δβ * ANetwind;
        double aΔα = +(aY - G*Δα - 2*Δα_vel*R_vel) / R;
        double aΔβ = -(aX + G*Δβ + 2*Δβ_vel*R_vel) / R;
        double aR  = -ANetwind + G;
        
        // OUTPUT: apply current velocity
        Y += dt * Y_vel;
        X += dt * X_vel;
        //R  += dt * R_vel;
        Δα += dt * Δα_vel;
        Δβ += dt * Δβ_vel;

        // compute next velocities  v = v0 + a * Δt
        Y_vel += dt * aY;
        X_vel += dt * aX;
        Δα_vel += dt * aΔα;
        Δβ_vel += dt * aΔβ;
        R_vel  += dt * aR;
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearConstantPendulum(double dt, double Frail, double Fcart)
    {
        PrepareBasicRelations(Frail, Fcart, 0.0);

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
    }
    
    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearCompleteModel(double dt, double Frail, double Fcart, double Fline)
    {
        double sA = sin(Alfa);
        double sB = sin(Beta);
        double cA = cos(Alfa);
        double cB = cos(Beta);
        PrepareBasicRelations(Frail, Fcart, Fline);

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
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearOriginalModel(double dt, double Frail, double Fcart, double Fline)
    {
        PrepareBasicRelations(Frail, Fcart, Fline);

        double sA = sin(Alfa); double sB = sin(Beta);
        double cA = cos(Alfa); double cB = cos(Beta);
        double sA2 = sA*sA; double sB2 = sB*sB;
        double μ1cA = μ1*cA;
        double μ2sAsB = μ2*sA*sB;
        double βv2 = Beta_vel*Beta_vel;

        double V5 = cA*sA*βv2*R - 2 * R_vel*Alfa_vel + G * cA*cB;
        double V6 = 2 * Beta_vel*(cA*Alfa_vel*R + sA * R_vel) + G * sB;
        double V7 = sA2*βv2*R + G * sA*cB + Alfa_vel * Alfa_vel*R;

        double Tsx = 5 / (Mcart + Mrail); // 1.490
        double Tsy = 7.5 / Mcart; // 6.493
        double Tsz = 10 / Mpayload; // 10

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
    }

    //////////////////////////////////////////////////////////////////////

    // dampen values that are very close to 0.0
    constexpr double Dampen(double x)
    {
        return abs(x) < 0.0000001 ? 0.0 : x;
    }

    constexpr double clamp(double x, double min, double max)
    {
        if (x <= min) return min;
        if (x >= max) return max;
        return x;
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
