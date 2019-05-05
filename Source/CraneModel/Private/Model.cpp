// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "Model.h"
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <sstream> // std::stringstream
#include <iomanip> // std::setprecision

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    // time between each discrete step of the simulation
    static constexpr double _PI = 3.14159265358979323846;
    static constexpr double _90degs = (_PI / 2);

    //////////////////////////////////////////////////////////////////////
    
    Model::Model(ModelType type) : Type{type}
    {
        Reset();
    }

    void Model::Reset()
    {
        Rail.Reset();
        Cart.Reset();
        Line.Reset();
        Alfa.Reset();
        Beta.Reset();

        Line.Pos = 0.5;
        Line.FrictionDir = -1.0;
        Line.CoeffStatic = 0.3;
        Line.CoeffKinetic = 0.2;

        Rail.VelMax = VelocityMax;
        Cart.VelMax = VelocityMax;
        Line.VelMax = VelocityMax;
        Alfa.VelMax = VelocityMax;
        Beta.VelMax = VelocityMax;

        if (Type == ModelType::Linear)
        {
            Alfa.SetLimits(-0.05, 0.05);
            Beta.SetLimits(-0.05, 0.05);
        }
        else
        {
            // Invert the alfa angle as required by non-linear models
            Alfa.Pos = _90degs;
            Alfa.SetLimits(0, _PI);
            Beta.SetLimits(-_90degs, _90degs);
        }
    }

    void Model::SetType(ModelType type)
    {
        if (Type != type) {
            Type = type;
            Reset();
        }
    }

    ModelState Model::GetState() const
    {
        // Restore the internal alfa representation
        ModelState s;
        s.RailOffset = Rail.Pos;
        s.CartOffset = Cart.Pos;
        s.LiftLine   = Line.Pos;

        // Formulas given by 3DCrane mathematical model description
        if (Type == ModelType::Linear)
        {
            s.Alfa = Alfa.Pos;
            s.Beta = Beta.Pos;
            s.PayloadX = s.RailOffset + s.LiftLine * s.Beta;
            s.PayloadY = s.CartOffset - s.LiftLine * s.Alfa;
            s.PayloadZ = -s.LiftLine;
        }
        else
        {
            // Calculate non-linear model payload position from Xw,Yw,α,β,R
            // Invert the alfa angle as required by non-linear models
            s.Alfa = Alfa.Pos;
            s.Beta = Beta.Pos;
            s.PayloadX = s.RailOffset + s.LiftLine * sin(s.Alfa) * sin(s.Beta);
            s.PayloadY = s.CartOffset + s.LiftLine * cos(s.Alfa);
            s.PayloadZ = -s.LiftLine * sin(s.Alfa) * cos(s.Beta);
        }
        return s;
    }

    static const char* ToString(ModelType type)
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

    static void format(std::wostream& out, const wchar_t* fmt, ...)
    {
        const int max = 2048;
        wchar_t buf[max];
        va_list ap; va_start(ap, fmt);
        int len = _vsnwprintf_s(buf, max, fmt, ap);
        if (len < 0) { buf[max - 1] = L'\0'; len = (int)wcslen(buf); }
        out.write(buf, len);
    }

    std::wstring Model::GetStateDebugText() const
    {
        ModelState state = GetState();
        std::wstringstream ss;
        format(ss, L"Model: %hs \n", ToString(Type));
        format(ss, L"  pos %+6.2f, %+6.2f, %+6.2f \n", state.PayloadX, state.PayloadY, state.PayloadZ);
        format(ss, L"  XYR %+6.2f, %+6.2f, %+6.2f \n", state.RailOffset, state.CartOffset, state.LiftLine);
        format(ss, L" vXYR %+6.2f, %+6.2f, %+6.2f m/s\n", Rail.Vel, Cart.Vel, Line.Vel);
        format(ss, L"  α  %+6.2f v %+6.2f rad/s \n", Alfa.Pos, Alfa.Vel);
        format(ss, L"  β  %+6.2f v %+6.2f rad/s \n", Beta.Pos, Beta.Vel);
        auto printComponent = [&](const char* which, const Component& c) {
            format(ss, L"  %hs a %+6.2f m/s², Fdrv %+5.1f, Fapp %+5.1f, Fst %+5.1f, Fki %+5.1f \n",
                which, c.Acc.Value, c.Fnet.Value, c.Applied.Value, c.SFriction.Value, c.KFriction.Value);
        };
        printComponent("Rail", Rail);
        printComponent("Cart", Cart);
        printComponent("Line", Line);
        format(ss, L"  iter# %5lld  dt %5.4f  iter/s %.1f \n", DiscreteStepCounter, DbgFixedTimeStep, DbgAvgIterations);
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
        SimulationTimeSink += deltaTime;
        int iterations = static_cast<int>(SimulationTimeSink / fixedTime);
        SimulationTimeSink -= iterations * fixedTime;

        DbgFixedTimeStep = fixedTime;
        DbgAvgIterations = (DbgAvgIterations + iterations) * 0.5;

        for (int i = 0; i < iterations; ++i)
        {
            Update(fixedTime, Frail, Fcart, Fwind);
        }

        // and finally, return the observable current state
        return GetState();
    }

    ModelState Model::Update(double deltaTime, Force Frail, Force Fcart, Force Fwind)
    {
        ++DiscreteStepCounter;
        switch (Type)
        {
            default:
            case ModelType::Linear:             BasicLinearModel(deltaTime, Frail, Fcart, Fwind); break;
            case ModelType::NonLinearConstLine: NonLinearConstLine(deltaTime, Frail, Fcart, Fwind); break;
            case ModelType::NonLinearComplete:  NonLinearCompleteModel(deltaTime, Frail, Fcart, Fwind); break;
            case ModelType::NonLinearOriginal:  NonLinearOriginalModel(deltaTime, Frail, Fcart, Fwind); break;
        }

        GetState().Print();

        // and finally, return the observable current state
        return GetState();
    }

    //////////////////////////////////////////////////////////////////////

    void Model::PrepareBasicRelations(Force Frail, Force Fcart, Force Fwind)
    {
        // prepare basic relations
        // we recalculate every frame to allow dynamically changing the model parameters

        // these are cable driven tension/friction coefficients:
        μ1 = Mpayload / Mcart;           // μ1 = Mc / Mw
        μ2 = Mpayload / (Mcart + Mrail); // μ2 = Mc / (Mw + Ms)

        // New friction model
        Rail.Mass = Mrail+Mcart;
        Cart.Mass = Mcart;
        Line.Mass = Mpayload;

        if (Type == ModelType::Linear)
        {
            Rail.ApplyForce(Frail, g);
            Cart.ApplyForce(Fcart, g);
            Line.ApplyForce(Fwind, g);
        }
        else
        {
            double Tsx = 5.0, Tsy = 7.5, Tsz = 10.0;
            Rail.ApplyForceNonLinear(Frail, g, RailFriction, Tsx);
            Cart.ApplyForceNonLinear(Fcart, g, CartFriction, Tsy);
            Line.ApplyForceNonLinear(Fwind, g, WindingFriction, Tsz);
        }
    }

    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        // calculate new force driven accelerations
        double R = Line.Pos;
        Accel aX = Rail.NetAcc + μ2 * Beta.Pos * Line.NetAcc;
        Accel aY = Cart.NetAcc - μ1 * Alfa.Pos * Line.NetAcc;
        Accel aR = g - Line.NetAcc;
        double line_vel = integrate_velocity(0.0, aR, dt);
        Accel aA =  (aY - g*Alfa.Pos - 2*Alfa.Vel*line_vel) / R;
        Accel aB = -(aX + g*Beta.Pos + 2*Beta.Vel*line_vel) / R;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Line.Const = (Fwind == 0.0);
        Line.Update(aR, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearConstLine(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double sA2 = sA*sA; double sB2 = sB*sB; double cA2 = cA*cA;

        double A = 1 + μ1 * cA2 + μ2 * sA2*sB2;
        double B = 1 + μ1;
        double R = Line.Pos;
        double G = g.Value;
        double a2 = Alfa.Vel*Alfa.Vel;
        double b2 = Beta.Vel*Beta.Vel;
        double V1 = μ1*R*cA*(a2 + sA2*b2) + μ1*G*cA*sA*cB;
        double V2 = μ2*R*sB*(sA*sA2*b2 + sA*a2) + μ2*G*sA2*cB*sB;
        double V3 = B*G*cA*cB + B*R*cA*sA*b2 + (μ1 - μ2*sB2)*R*cA*sA*a2;
        double V4 = μ2*R*sA*cB*sB*(a2 + sA2*b2) + (B + (μ2 - μ1)*sA2)*G*sB + 2*R*A*cA*Alfa.Vel*Beta.Vel;

        Accel aX = (-μ2*cA*sA*sB*Cart.NetAcc + (1 + μ1*cA2)*Rail.NetAcc + V2) / A;
        Accel aY = ((1 + μ2*sA2*sB2)*Cart.NetAcc - μ1*cA*sA*sB*Rail.NetAcc + V1) / A;
        Accel aA = ((1 + μ2*sB2)*sA*Cart.NetAcc - B*cA*sB*Rail.NetAcc + V3) / (R*A);
        Accel aB = (μ2*cA*sA*cB*sB*Cart.NetAcc - (B - μ1 * sA2)*cB*Rail.NetAcc - V4) / (R*A*sA);

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
    }
    
    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearCompleteModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);

        double R = Line.Pos;
        double G = g.Value;
        double b2 = Beta.Vel*Beta.Vel;
        double V5 = cA*sA*b2*R - 2*Line.Vel*Alfa.Vel + G*cA*cB;
        double V6 = 2*Beta.Vel*(cA*Alfa.Vel*R + sA*Line.Vel) + G*sB;
        double V7 = sA*sA*b2*R + G*sA*cB + Alfa.Vel*Alfa.Vel*R;

        Accel aX = Rail.NetAcc + μ2 * Line.NetAcc * sA * sB;
        Accel aY = Cart.NetAcc + μ1 * Line.NetAcc * cA;
        Accel aA =  (sA*Cart.NetAcc - cA*sB*Rail.NetAcc + (μ1 - μ2*sB*sB)*cA*sA*Line.NetAcc + V5) / R;
        Accel aB = -(cB*Rail.NetAcc + μ2*sA*cB*sB*Line.NetAcc + V6) / (sA*R);
        Accel aR =  -cA*Cart.NetAcc - sA*sB*Rail.NetAcc - (1 + μ1*cA*cA + μ2*sA*sA*sB*sB)*Line.NetAcc + V7;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Line.Update(aR, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearOriginalModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double sA2 = sA*sA,   sB2 = sB*sB;
        double μ1cA = μ1*cA;
        double μ2sAsB = μ2*sA*sB;
        double βv2 = Beta.Vel*Beta.Vel;
        double R = Line.Pos;
        double G = g.Value;
        double V5 = cA*sA*βv2*R - 2 * Line.Vel*Alfa.Vel + G*cA*cB;
        double V6 = 2 * Beta.Vel*(cA*Alfa.Vel*R + sA * Line.Vel) + G*sB;
        double V7 = sA2*βv2*R + G*sA*cB + Alfa.Vel * Alfa.Vel*R;

        Accel ADrrail = Frail / (Mcart + Mpayload); // u2 = Fx / (Mw + Mc) | rail driving accel
        Accel ADrcart = Fcart / Mcart;              // u1 = Fy / Mw        | cart driving accel
        Accel ADrwind = Fwind / Mpayload;           // u3 = Fr / Mc        | wind driving accel

        Accel AFrrail = Force{RailFriction} / (Mcart + Mpayload); // T2 = Tx / (Mw + Mc) | rail friction accel
        Accel AFrcart = Force{CartFriction} / Mcart;              // T1 = Ty / Mw        | cart friction accel
        Accel AFrwind = Force{WindingFriction} / Mpayload;        // T3 = Tr / Mc        | line winding friction accel

        Accel Tsx = Force{5.0} / (Mcart + Mrail); // 1.490
        Accel Tsy = Force{7.5} / Mcart; // 6.493
        Accel Tsz = Force{10.} / Mpayload; // 10
        Accel railFr = (Rail.Vel * AFrrail + Tsx * sign(Rail.Vel)); // some sort of rail friction accel
        Accel cartFr = (Cart.Vel * AFrcart + Tsy * sign(Cart.Vel)); // some sort of cart friction accel
        Accel windFr = -(Line.Vel * AFrwind + Tsz * sign(Line.Vel)); // NET winding accel
        Accel RailNetAcc = ADrrail - railFr; // N2 = rail net accel
        Accel CartNetAcc = ADrcart - cartFr; // N1 = cart net accel
        Accel WindNetAcc = ADrwind - windFr; // N3 = line net accel

        Accel aX = ADrrail - railFr + μ2sAsB*ADrwind - μ2sAsB*windFr;
        Accel aY = ADrcart - cartFr + μ1cA*ADrwind - μ1cA*windFr;
        Accel aA = (-cartFr*sA
            + ADrcart * sA + cA*R * sA*βv2 - cA*ADrrail * sB + cA*cB*g
            - cA*μ2sAsB*sB*ADrwind + cA*μ2sAsB*sB*windFr + cA*railFr*sB
            + μ1cA*ADrwind * sA - μ1cA*windFr*sA - 2 * Line.Vel * Alfa.Vel
            ) / R;
        Accel aB = -(
            cB*ADrrail + g*sB + 2*R * cA*Alfa.Vel*Beta.Vel
            + cB*μ2sAsB*ADrwind - cB*μ2sAsB*windFr
            + 2 * Line.Vel * sA*Beta.Vel - cB*railFr
            ) / (R*sA);
        Accel aR = cA*cartFr - cA*ADrcart
            + R * sA2*βv2 - ADrrail * sA*sB
            + sA*cB*g
            - μ2sAsB*sA*sB*ADrwind + μ2sAsB*sA*sB*windFr
            + railFr*sA*sB
            - μ1*ADrwind + μ1*ADrwind*sA2
            + μ1 * windFr - μ1 * windFr*sA2
            + R * Alfa.Vel * Alfa.Vel - ADrwind + windFr;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Line.Update(aR, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
    }

    //////////////////////////////////////////////////////////////////////
}
