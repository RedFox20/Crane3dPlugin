// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#include "Model.h"
#include <cmath>
#include <cstdio>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    // time between each discrete step of the simulation
    static constexpr double SimulationStep = 0.001;
    static constexpr double _90degs = (3.1415926 / 2);

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
        s.RailOffset = Xw;
        s.CartOffset = Yw;
        s.LiftLine = R;

        // Formulas given by 3DCrane mathematical model description
        if (Type == ModelType::Linear)
        {
            s.Alfa = Δα;
            s.Beta = Δβ;
            s.PayloadX = Xw + R*Δβ;
            s.PayloadY = Yw - R*Δα;
            s.PayloadZ = -R;
        }
        else
        {
            // Calculate payload position from Xw,Yw,α,β,R
            double α = _90degs - Alfa;
            s.Alfa = α;
            s.Beta = Beta;
            s.PayloadX = Xw + R * sin(α) * sin(Beta);
            s.PayloadY = Yw + R * cos(α);
            s.PayloadZ = -R * sin(α) * cos(Beta);
        }
        return s;
    }

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
                case ModelType::NonLinearConstantLine:
                    NonLinearConstantPendulum(SimulationStep, Frail, Fcart);
                    break;
                case ModelType::NonLinearComplete:
                    NonLinearCompleteModel(SimulationStep, Frail, Fcart, Fline);
                    break;
            }
        }

        ApplyLimits();

        // and finally, return the observable current state
        return GetState();
    }

    //////////////////////////////////////////////////////////////////////

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

        N1 = u1 - T1; // cart net acceleration force
        N2 = u2 - T2; // rail net acceleration force
        N3 = u3 - T3; // line net acceleration force

        μ1 = Mpayload / Mcart;           // μ1 = Mc / Mw
        μ2 = Mpayload / (Mcart + Mrail); // μ2 = Mc / (Mw + Ms)
    }

    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, double Frail, double Fcart)
    {
        PrepareBasicRelations(Frail, Fcart, 0.0);

        // calculate accelerations:
        double aYw = N1 - μ1*Δα*N3;
        double aXw = N2 + μ2*Δβ*N3;
        double aΔα =  (N1 - μ1*Δα*N3 - G*Δα - 2*Δα_vel*R_vel) / R;
        double aΔβ = -(N2 + μ2*Δβ*N3 + G*Δβ + 2*Δβ_vel*R_vel) / R;
        double aR  = -N3 + G;
        
        // OUTPUT: apply current velocity
        Yw += dt * Yw_vel;
        Xw += dt * Xw_vel;
        R  += dt * R_vel;
        Δα += dt * Δα_vel;
        Δβ += dt * Δβ_vel;

        // compute next velocities  v = a*t
        Yw_vel += dt * aYw;
        Xw_vel += dt * aXw;
        Δα_vel += dt * aΔα;
        Δβ_vel += dt * aΔβ;
        R_vel  += dt * aR;
    }

    //////////////////////////////////////////////////////////////////////

    void Model::PrepareNonLinearState()
    {
        x1 = Yw; x2 = Yw_vel;
        x3 = Xw; x4 = Xw_vel;
        x5 = Alfa; x6 = Alfa_vel;
        x7 = Beta; x8 = Beta_vel;
        x9 = R;    x10 = R_vel;
        s5 = sin(x5); s7 = sin(x7);
        c5 = cos(x5); c7 = cos(x7);
    }

    void Model::DeriveNonLinearOutput(double dt, bool deriveLiftLine)
    {
        // OUTPUT: apply Euler method for integrating the derived d1..d10
        Yw += dt * d1;
        Xw += dt * d3;
        Yw_vel += dt * d2;
        Xw_vel += dt * d4;

        Alfa += dt * d5;
        Beta += dt * d7;
        Alfa_vel += dt * d6;
        Beta_vel += dt * d8;

        if (deriveLiftLine)
        {
            R += dt * d9;
            R_vel += dt * d10;
        }
    }

    void Model::NonLinearConstantPendulum(double dt, double Frail, double Fcart)
    {
        PrepareNonLinearState();

        double A = 1 + μ1 * c5*c5 + μ2 * s5*s5*s7*s7;
        double B = 1 + μ1;
        double V1 = μ1*R*c5*(x6*x6 + s5*s5*x8*x8) + μ1*G*c5*s5*c7;
        double V2 = μ2*R*s7*(s5*s5*s5*x8*x8 + s5*x6*x6) + μ2*G*s5*s5*c7*s7;
        double V3 = B*G*c5*c7 + B*R*c5*s5*x8*x8 + (μ1 - μ2*s7*s7)*R*c5*s5*x6*x6;
        double V4 = μ2*R*s5*c7*s7*(x6*x6 + s5*s5*x8*x8) + (B + (μ2 - μ1)*s5*s5)*G*s7 + 2*R*A*c5*x6*x8;

        // derived x1..x8 as per 3DCrane mathematical model description
        d1 = x2;
        d2 = ((1 + μ2*s5*s5*s7*s7)*N1 - μ1*c5*s5*s7*N2 + V1) / A;
        d3 = x4;
        d4 = (-μ2*c5*s5*s7*N1 + (1 + μ1*c5*c5)*N2 + V2) / A;
        d5 = x6;
        d6 = ((1 + μ2*s7*s7)*s5*N1 - B*c5*s7*N2 + V3) / (R*A);
        d7 = x8;
        d8 = 0.0; // if s5 is 0.0, then there is no Beta velocity
        if (abs(s5) > 0.000001) {
            d8 = (μ2*c5*s5*c7*s7*N1 - (B - μ1 * s5*s5)*c7*N2 - V4) / (R*A*s5);
        }

        DeriveNonLinearOutput(dt, false);
    }
    
    //////////////////////////////////////////////////////////////////////

    constexpr double sign(double x)
    {
        return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0);
    }

    void Model::NonLinearCompleteModel(double dt, double Frail, double Fcart, double Fline)
    {
        PrepareNonLinearState();

        // NOTE: variable names left exactly the same as the original math. model descr.
        double V5 = c5 * s5*x8*x8*x9 - 2 * x10*x6 + G * c5*c7;
        double V6 = 2 * x8*(c5*x6*x9 + s5 * x10) + G * s7;
        double V7 = s5 * s5*x8*x8*x9 + G * s5*c7 + x6 * x6*x9;

        // derived x1..x10 as per 3DCrane mathematical model description
        //d1 = x2;
        //d2 = N1 + μ1 * c5 * N3;
        //d3 = x4;
        //d4 = N2 + μ2 * s5 * s7 * N3;
        //d5 = x6;
        //d6 = (s5 * N1 - c5 * s7 * N2 + (μ1 - μ2 * s7*s7) * c5*s5*N3 * V5) / x9;
        //d7 = x8;
        //d8 = 0.0; // if s5 is 0.0, then there is no Beta velocity
        //if (abs(s5) > 0.000001) {
        //	d8 = -(c7*N2 + μ2 * s5*c7*s7*N3 + V6) / (s5*x9);
        //}
        //d9 = x10;
        //d10 = -c5 * N1 - s5 * s7*N2 - (1 + μ1 * c5*c5 + μ2 * s5*s5*s7*s7)*N3 + V7;

        // original model:
        double Tsx = 5 / (Mcart + Mrail);
        double Tsy = 7.5 / Mcart;
        double Tsz = 10 / Mpayload;
        d1 = x2;
        d2 = u1 - x2 * T1 - Tsy * sign(x2) + μ1 * c5*u3 - μ1 * c5*(-x10 * T3 - Tsz * sign(x10));
        d3 = x4;
        d4 = u2 - x4 * T2 - Tsx * sign(x4) + μ2 * s5*s7*u3 - μ2 * s5*s7*(-x10 * T3 - Tsz * sign(x10));
        d5 = x6;
        d6 = (-(x2 * T1 + Tsy * sign(x2))*s5 + u1 * s5 + c5*x9 * s5*x8 * x8 - c5*u2 * s7 + c5*c7*G - c5*μ2*s5*s7*s7*u3 + c5*μ2*s5*s7*s7*(-x10 * T3 - Tsz * sign(x10)) + c5*(x4 * T2 + Tsx * sign(x4))*s7 + c5*μ1*u3 * s5 - c5*μ1*(-x10 * T3 - Tsz * sign(x10))*s5 - 2 * x10 * x6) / x9;
        d7 = x8;
        d8 = -(c7*u2 + G * s7 + 2 * x9 * c5*x6 * x8 + c7*μ2*s5*s7*u3 - c7*μ2*s5*s7*(-x10 * T3 - Tsz * sign(x10)) + 2 * x10 * s5*x8 - c7*(x4 * T2 + Tsx * sign(x4))) / (x9 * s5);
        d9 = x10;
        d10 = c5*(x2 * T1 + Tsy * sign(x2)) - c5*u1 + x9 * s5*s5*x8 * x8 - u2 * s5*s7 + s5*c7*G - μ2 * s5*s5*s7*s7*u3 + μ2 * s5*s5*s7*s7*(-x10 * T3 - Tsz * sign(x10)) + (x4 * T2 + Tsx * sign(x4))*s5*s7 - μ1 * u3 + μ1 * u3 * s5*s5 + μ1 * (-x10 * T3 - Tsz * sign(x10)) - μ1 * (-x10 * T3 - Tsz * sign(x10))*s5*s5 + x9 * x6 * x6 - u3 - x10 * T3 - Tsz * sign(x10);

        DeriveNonLinearOutput(dt, true);
        //DampenAllValues();
    }

    //////////////////////////////////////////////////////////////////////

    // dampen values that are very close to 0.0
    constexpr double Dampen(double x)
    {
        return abs(x) < 0.0000001 ? 0.0 : x;
    }

    constexpr double clamp(double x, double min, double max)
    {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

    void Model::ApplyLimits()
    {
        Xw = clamp(Xw, RailLimitMin, RailLimitMax);
        Yw = clamp(Yw, CartLimitMin, CartLimitMax);
        R  = clamp(R, LineLimitMin, LineLimitMax);
    }

    void Model::DampenAllValues()
    {
        Yw   = Dampen(Yw);
        Xw   = Dampen(Xw);
        Alfa = Dampen(Alfa);
        Beta = Dampen(Beta);
        R    = Dampen(R);

        Yw_vel   = Dampen(Yw_vel);
        Xw_vel   = Dampen(Xw_vel);
        Alfa_vel = Dampen(Alfa_vel);
        Beta_vel = Dampen(Beta_vel);
        R_vel    = Dampen(R_vel);
    }

    //////////////////////////////////////////////////////////////////////

    void ModelState::Print() const
    {
        printf("Alfa: %.2f Beta: %.2f\n"
            "Rail: %.2f Cart: %.2f Line: %.2f\n"
            "X: %.2f Y: %.2f Z: %.2f\n",
            Alfa, Beta,
            RailOffset, CartOffset, LiftLine,
            PayloadX, PayloadY, PayloadZ);
    }

    //////////////////////////////////////////////////////////////////////
}
