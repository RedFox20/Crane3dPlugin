// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
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

    const Force Force::Zero { 0.0 };
    const Mass  Mass::Zero  { 0.0 };
    const Accel Accel::Zero { 0.0 };

    constexpr double clamp(double x, double min, double max)
    {
        if (x <= min) return min;
        if (x >= max) return max;
        return x;
    }
    
    // dampen values that are very close to 0.0
    constexpr double Dampen(double x)
    {
        return std::abs(x) < 0.001 ? 0.0 : x;
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

        // Formulas given by 3DCrane mathematical model description
        if (Type == ModelType::Linear)
        {
            s.RailOffset = Rail.Pos;
            s.CartOffset = Cart.Pos;
            s.LiftLine   = Line.Pos;
            s.Alfa = CAlfa.Pos;
            s.Beta = CBeta.Pos;
            s.PayloadX = s.RailOffset + s.LiftLine * s.Beta;
            s.PayloadY = s.CartOffset - s.LiftLine * s.Alfa;
            s.PayloadZ = -s.LiftLine;
        }
        else
        {
            s.RailOffset = X;
            s.CartOffset = Y;
            s.LiftLine   = R;
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

    void format(std::wostream& out, const wchar_t* fmt, ...)
    {
        const int max = 1024;
        wchar_t buf[max];
        va_list ap; va_start(ap, fmt);
        int len = _vsnwprintf_s(buf, _TRUNCATE, fmt, ap);
        if (len < 0) { len = max-1; }
        out.write(buf, len);
    }

    std::wstring Model::GetStateDebugText() const
    {
        ModelState state = GetState();
        std::wstringstream ss;
        format(ss, L"Model: %hs \n", ToString(Type));
        format(ss, L"  pos { %+6.2f, %+6.2f, %+6.2f }\n", state.PayloadX, state.PayloadY, state.PayloadZ);
        format(ss, L"  XYR { %+6.2f, %+6.2f, %+6.2f }\n", state.RailOffset, state.CartOffset, state.LiftLine);
        if (Type == ModelType::Linear)
        {
            format(ss, L" vXYR { %+6.2f, %+6.2f, %+6.2f } m/s\n", Rail.Vel, Cart.Vel, Line.Vel);
            format(ss, L"  Δα    %+6.2f  v %+6.2f rad/s \n", CAlfa.Pos, CAlfa.Vel);
            format(ss, L"  Δβ    %+6.2f  v %+6.2f rad/s \n", CBeta.Pos, CBeta.Vel);
        }
        else
        {
            format(ss, L" vXYR { %+6.2f, %+6.2f, %+6.2f } m/s\n", X_vel, Y_vel, R_vel);
            format(ss, L"  α  %+6.2f  v %+6.2f rad/s \n", Alfa, Alfa_vel);
            format(ss, L"  β  %+6.2f  v %+6.2f rad/s \n", Beta, Beta_vel);
        }
        auto printComponent = [&](const char* which, const Component& c) {
            format(ss, L"  %hs a %+6.2f m/s², Fnet %+5.1f, Fapp %+5.1f, Fri %+5.1f \n",
                which, c.NetAcc.Value, c.Fnet.Value, c.Applied.Value, c.Friction.Value);
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

    void Component::Update(Accel new_acc, double dt)
    {
        double newPos = Pos + Vel * dt + Acc.Value*(dt*dt*0.5);
        newPos = clamp(newPos, LimitMin, LimitMax);

        // since position can be clamped, we calculate avg velocity instead
        Vel = average_velocity(Pos, newPos, dt);
        Pos = newPos;
        Acc = new_acc;

        Vel = Dampen(Vel); // dampen extremely small velocities
    }

    void Component::ApplyForce(Force applied, Accel g)
    {
        Force Fnormal = Mass * g; // normal force between body and surface
        Friction = Force::Zero;
        if (std::abs(Vel) < 0.001) // static friction
        {
            Force staticMax = CoeffStatic * Fnormal;
            if (abs(applied) > staticMax) // resist up to max static friction
                Friction = sign(applied) * (abs(applied) - staticMax);
            else
                Friction = applied; // cancel out applied force
        }
        else
        {
            Force kinetic = CoeffKinetic * Fnormal;
            Friction = sign(Vel) * kinetic;
        }

        // @note Fnet positive: driving dominates
        // @note Fnet negative: friction dominates
        Applied = applied;
        Fnet = applied - Friction;
        Fnet = ClampForceByPosLimits(Fnet); // cannot accelerate when stuck
        NetAcc = Fnet / Mass;
    }

    Force Component::ClampForceByPosLimits(Force force) const
    {
        if (force > 0.0 && Pos >= LimitMax) return Force::Zero;
        if (force < 0.0 && Pos <= LimitMin) return Force::Zero;
        return force;
    }

    //////////////////////////////////////////////////////////////////////

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

        ADrcart = Fcart.Value / Mcart.Value;              // u1 = Fy / Mw        | cart driving accel
        ADrrail = Frail.Value / (Mcart + Mpayload).Value; // u2 = Fx / (Mw + Mc) | rail driving accel
        ADrwind = Fwind.Value / Mpayload.Value;           // u3 = Fr / Mc        | wind driving accel

        AFrcart = CartFriction / Mcart.Value;              // T1 = Ty / Mw        | cart friction accel
        AFrrail = RailFriction / (Mcart + Mpayload).Value; // T2 = Tx / (Mw + Mc) | rail friction accel
        AFrwind = WindingFriction / Mpayload.Value;        // T3 = Tr / Mc        | line winding friction accel

        double Tsx = 5 / (Mcart + Mrail).Value; // 1.490
        double Tsy = 7.5 / Mcart.Value; // 6.493
        double Tsz = 10 / Mpayload.Value; // 10
        double railFr = (X_vel * AFrrail + Tsx * sign(X_vel)); // some sort of rail friction accel
        double cartFr = (Y_vel * AFrcart + Tsy * sign(Y_vel)); // some sort of cart friction accel
        double aWind = -(R_vel * AFrwind + Tsz * sign(R_vel)); // NET winding accel
        N1 = ADrcart - cartFr; // cart net accel
        N2 = ADrrail - railFr; // rail net accel
        N3 = ADrwind - aWind; // line net accel

        // these are cable driven friction coefficients:
        μ1 = Mpayload / Mcart;           // μ1 = Mc / Mw
        μ2 = Mpayload / (Mcart + Mrail); // μ2 = Mc / (Mw + Ms)

        // New friction model
        Mass Mcartpayload = Mcart+Mpayload;
        Mass Mall = Mrail+Mcart+Mpayload;

        Rail.Mass = Mrail+Mcart+Mpayload;
        Cart.Mass = Mcart+Mpayload;
        Line.Mass = Mpayload;

        Rail.ApplyForce(Frail, g);
        Cart.ApplyForce(Fcart, g);
        Line.ApplyForce(Fwind, g);
    }

    //////////////////////////////////////////////////////////////////////

    // This simplified model assumes that α and β are very small
    void Model::BasicLinearModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        // calculate new force driven accelerations
        Accel aX = Rail.NetAcc + μ2 * CBeta.Pos * Line.NetAcc;
        Accel aY = Cart.NetAcc - μ1 * CAlfa.Pos * Line.NetAcc;
        Accel aR = g - Line.NetAcc;
        Accel aA =  (aY - g*CAlfa.Pos - 2*CAlfa.Vel*Line.Vel) / R;
        Accel aB = -(aX + g*CBeta.Pos + 2*CBeta.Vel*Line.Vel) / R;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Line.Update(aR, dt);
        CAlfa.Update(aA, dt);
        CBeta.Update(aB, dt);
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

        X = clamp(X, Rail.LimitMin, Rail.LimitMax);
        Y = clamp(Y, Cart.LimitMin, Cart.LimitMax);
        R = clamp(R, Line.LimitMin, Line.LimitMax);
        Alfa = clamp(Alfa, -_PI, +_PI);
        Beta = clamp(Beta, -_PI, +_PI);
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

        X = clamp(X, Rail.LimitMin, Rail.LimitMax);
        Y = clamp(Y, Cart.LimitMin, Cart.LimitMax);
        R = clamp(R, Line.LimitMin, Line.LimitMax);
        Alfa = clamp(Alfa, -_PI, +_PI);
        Beta = clamp(Beta, -_PI, +_PI);
    }

    //////////////////////////////////////////////////////////////////////

    void Model::NonLinearOriginalModel(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        PrepareBasicRelations(Frail, Fcart, Fwind);

        double sA = sin(Alfa), sB = sin(Beta);
        double cA = cos(Alfa), cB = cos(Beta);
        double sA2 = sA*sA,   sB2 = sB*sB;
        double μ1cA = μ1*cA;
        double μ2sAsB = μ2*sA*sB;
        double βv2 = Beta_vel*Beta_vel;

        double V5 = cA*sA*βv2*R - 2 * R_vel*Alfa_vel + G * cA*cB;
        double V6 = 2 * Beta_vel*(cA*Alfa_vel*R + sA * R_vel) + G * sB;
        double V7 = sA2*βv2*R + G * sA*cB + Alfa_vel * Alfa_vel*R;

        double Tsx = 5 / (Mcart + Mrail).Value; // 1.490
        double Tsy = 7.5 / Mcart.Value; // 6.493
        double Tsz = 10 / Mpayload.Value; // 10

        AFrrail = RailFriction / (Mcart + Mpayload).Value; // T2 = Tx / (Mw + Mc) | rail friction accel
        AFrcart = CartFriction / Mcart.Value;              // T1 = Ty / Mw        | cart friction accel
        AFrwind = WindingFriction / Mpayload.Value;        // T3 = Tr / Mc        | line winding friction accel

        double railFr = (X_vel * AFrrail + Tsx * sign(X_vel)); // some sort of rail friction accel
        double cartFr = (Y_vel * AFrcart + Tsy * sign(Y_vel)); // some sort of cart friction accel
        double aWind = -(R_vel * AFrwind + Tsz * sign(R_vel)); // NET winding accel

        double aX = ADrrail - railFr + μ2sAsB*u3 - μ2sAsB*aWind;
        double aY = ADrcart - cartFr + μ1cA*u3 - μ1cA*aWind;
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

        X = integrate_pos(X, X_vel, Accel{aX}, dt);
        Y = integrate_pos(Y, Y_vel, Accel{aY}, dt);
        R = integrate_pos(R, R_vel, Accel{aR}, dt);
        Alfa = integrate_pos(Alfa, Alfa_vel, Accel{aAlfa}, dt);
        Beta = integrate_pos(Beta, Beta_vel, Accel{aBeta}, dt);

        X_vel = integrate_velocity(X_vel, Accel{aX}, dt);
        Y_vel = integrate_velocity(Y_vel, Accel{aY}, dt);
        R_vel = integrate_velocity(R_vel, Accel{aR}, dt);
        Alfa_vel = integrate_velocity(Alfa_vel, Accel{aAlfa}, dt);
        Beta_vel = integrate_velocity(Beta_vel, Accel{aBeta}, dt);

        X = clamp(X, Rail.LimitMin, Rail.LimitMax);
        Y = clamp(Y, Cart.LimitMin, Cart.LimitMax);
        R = clamp(R, Line.LimitMin, Line.LimitMax);
        Alfa = clamp(Alfa, -_PI, +_PI);
        Beta = clamp(Beta, -_PI, +_PI);
    }

    //////////////////////////////////////////////////////////////////////
}
