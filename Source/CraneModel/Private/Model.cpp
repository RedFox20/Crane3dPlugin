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
    static constexpr double _180degs = _PI;
    static constexpr double _90degs = (_180degs / 2);
    static constexpr double _60degs = _180degs/3.0;
    static constexpr double _45degs = _180degs/4.0;
    static constexpr double _30degs = _180degs/6.0;

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

        Rail.AccMax = AccelMax;
        Cart.AccMax = AccelMax;
        Line.AccMax = AccelMax;

        Rail.CoeffStaticColoumb = 5.0;
        Cart.CoeffStaticColoumb = 7.5;
        Line.CoeffStaticColoumb = 10.0;

        Rail.CoeffKineticViscous = 100.0;
        Cart.CoeffKineticViscous = 82.0;
        Line.CoeffKineticViscous = 75.0;

        if (Type == ModelType::Linear)
        {
            Alfa.SetLimits(-0.1, +0.1);
            Beta.SetLimits(-0.1, +0.1);
        }
        else
        {
            // Invert the alfa angle as required by non-linear models
            Alfa.Pos = _90degs;
            // the crane cannot physically swing more than X degrees due to cart edges
            Alfa.SetLimits(_45degs, _180degs - _45degs);
            Beta.SetLimits(-_45degs, _45degs);
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
        double A = Alfa.Pos;
        double B = Beta.Pos;
        ModelState s;
        s.RailOffset = Rail.Pos;
        s.CartOffset = Cart.Pos;
        s.LiftLine   = Line.Pos;
        if (Type == ModelType::Linear)
        {
            s.PayloadX = s.RailOffset + s.LiftLine * B;
            s.PayloadY = s.CartOffset - s.LiftLine * A;
            s.PayloadZ = -s.LiftLine;
        }
        else
        {
            s.PayloadX = s.RailOffset + s.LiftLine * sin(A) * sin(B);
            s.PayloadY = s.CartOffset + s.LiftLine * cos(A);
            s.PayloadZ = -s.LiftLine * sin(A) * cos(B);
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
        format(ss, L" vXYR %+6.2f, %+6.2f, %+6.2f m/s \n", Rail.Vel, Cart.Vel, Line.Vel);
        format(ss, L"  α  %+6.2f  vα %+6.2f rad/s  aα %+6.2f rad/s^2 \n", Alfa.Pos, Alfa.Vel, Alfa.Acc.Value);
        format(ss, L"  β  %+6.2f  vβ %+6.2f rad/s  aβ %+6.2f rad/s^2 \n", Beta.Pos, Beta.Vel, Beta.Acc.Value);
        auto printComponent = [&](const char* which, const Component& c) {
            format(ss, L"  %hs a %+6.2f m/s², Fnet %+5.1f, Fapp %+5.1f, Fst %+5.1f, Fki %+5.1f \n",
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
        printf("Rail: %.2f Cart: %.2f Line: %.2f X: %.2f Y: %.2f Z: %.2f\n",
                RailOffset, CartOffset, LiftLine, PayloadX, PayloadY, PayloadZ);
    }

    //////////////////////////////////////////////////////////////////////

    ModelState Model::UpdateFixed(double fixedTime, double elapsedTime,
                                  Force Frail, Force Fcart, Force Fwind)
    {
        // we run the simulation with a constant time step
        SimulationTimeSink += elapsedTime;
        int iterations = static_cast<int>(SimulationTimeSink / fixedTime);
        SimulationTimeSink -= iterations * fixedTime;

        DbgFixedTimeStep = fixedTime;
        DbgAvgIterations = (DbgAvgIterations + iterations) * 0.5;

        for (int i = 0; i < iterations; ++i)
        {
            Update(fixedTime, Frail, Fcart, Fwind);
        }

        return GetState();
    }

    void Model::Update(double fixedTime, Force Frail, Force Fcart, Force Fwind)
    {
        ++DiscreteStepCounter;

        // cable driven tension/friction coefficients:
        μ1 = Mpayload / Mcart;           // μ1 = Mc / Mw
        μ2 = Mpayload / (Mcart + Mrail); // μ2 = Mc / (Mw + Ms)

        Rail.Mass = Mrail+Mcart;
        Cart.Mass = Mcart;
        Line.Mass = Mpayload;

        switch (Type)
        {
            default:
            case ModelType::Linear:
                BasicLinearModel(fixedTime, Frail, Fcart, Fwind);
                break;
            case ModelType::NonLinearComplete:
                NonLinearCompleteModel(fixedTime, Frail, Fcart, Fwind);
                break;
            case ModelType::NonLinearConstLine:
                NonLinearConstLine(fixedTime, Frail, Fcart, Fwind);
                break;
            case ModelType::NonLinearOriginal:
                NonLinearOriginalModel(fixedTime, Frail, Fcart, Fwind);
                break;
        }
        GetState().Print();
    }

    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        double R = Line.Pos;
        Accel aX = Rail.NetAcc + Line.NetAcc*μ2*Beta.Pos;
        Accel aY = Cart.NetAcc - Line.NetAcc*μ1*Alfa.Pos;
        Accel aA =  (aY - g*Alfa.Pos - 2*Alfa.Vel*Line.Vel) / R;
        Accel aB = -(aX + g*Beta.Pos + 2*Beta.Vel*Line.Vel) / R;
        Accel aR = g - Line.NetAcc;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
        Line.Update(aR, dt);
        if (Fwind == 0.0)
            Line.Pos = R;
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearConstLine(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        double R = Line.Pos;
        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double sA2 = sA*sA, sB2 = sB*sB, cA2 = cA*cA;

        double A = 1 + μ1 * cA2 + μ2 * sA2*sB2;
        double B = 1 + μ1;
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
        if (Fwind != 0.0)
            Line.Update(Line.NetAcc - g, dt);
    }
    
    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearCompleteModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double R = Line.Pos;
        double G = g.Value;
        double vB2 = Beta.Vel*Beta.Vel;
        double VA = R*vB2*cA*sA - 2*Line.Vel*Alfa.Vel + G*cA*cB;
        double VB = 2*Beta.Vel*(R*Alfa.Vel*cA + Line.Vel*sA) + G*sB;
        double VR = R*vB2*sA*sA + G*sA*cB + R*Alfa.Vel*Alfa.Vel;

        Accel aX = Rail.NetAcc + Line.NetAcc*μ2*sA*sB;
        Accel aY = Cart.NetAcc + Line.NetAcc*μ1*cA;
        Accel aA = (Cart.NetAcc*sA - Rail.NetAcc*cA*sB +
                    (μ1 - μ2*sB*sB)*Line.NetAcc*cA*sA + VA) / R;
        Accel aB = -(Rail.NetAcc*cB + Line.NetAcc*μ2*sA*cB*sB + VB) / (R*sA);
        Accel aR = - Cart.NetAcc*cA - Rail.NetAcc*sA*sB
                   - Line.NetAcc*(1 + μ1*cA*cA + μ2*sA*sA*sB*sB) + VR;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
        Line.Update(aR, dt);
        if (Fwind == 0.0)
            Line.Pos = R;
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearOriginalModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double sA2 = sA*sA, sB2 = sB*sB;
        double μ1cA = μ1*cA;
        double μ2sAsB = μ2*sA*sB;
        double vB2 = Beta.Vel*Beta.Vel;
        double R = Line.Pos;
        double G = g.Value;
        double VA = R*vB2*cA*sA - 2*Line.Vel*Alfa.Vel + G*cA*cB;
        double VB = 2*Beta.Vel*(R*Alfa.Vel*cA + Line.Vel*sA) + G*sB;
        double VR = R*vB2*sA2 + G*sA*cB + R*Alfa.Vel*Alfa.Vel;

        Accel aX = Rail.NetAcc + Line.NetAcc*μ2sAsB;
        Accel aY = Cart.NetAcc + Line.NetAcc*μ1cA;
        Accel aA = (Cart.NetAcc*sA - Rail.NetAcc*cA*sB
                    - Line.NetAcc*cA*μ2sAsB*sB + Line.NetAcc*μ1cA*sA + VA) / R;
        Accel aB = -(Rail.NetAcc*cB + g*sB + 2*R*cA*Alfa.Vel*Beta.Vel +
                       Line.NetAcc*cB*μ2sAsB + 2*sA*Line.Vel*Beta.Vel) / (R*sA);
        Accel aR = -Cart.NetAcc*cA - Rail.NetAcc*sA*sB
                     - μ2sAsB*sA*sB*Cart.NetAcc - Line.NetAcc*μ1
                    + Line.NetAcc*μ1*sA2 - Line.NetAcc + VR;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
        if (Fwind != 0.0)
            Line.Update(aR, dt);
    }

    //////////////////////////////////////////////////////////////////////
}
