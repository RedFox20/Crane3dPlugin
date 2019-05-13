// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "ModelImplementation.h"
#include "Model.h"

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    IModelImplementation::IModelImplementation(crane3d::Model& model)
        : Model{model}, Rail{model.Rail}, Cart{model.Cart}, 
          Line{model.Line}, Alfa{model.Alfa}, Beta{model.Beta}, g{model.g}
    {
    }

    //////////////////////////////////////////////////////////////////////

    void BasicLinearModel::Update(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        // cable driven tension/friction coefficients:
        double μ1 = Model.Mpayload / Cart.Mass; // μ1 = Mc / Mw
        double μ2 = Model.Mpayload / Rail.Mass; // μ2 = Mc / (Mw + Ms)
        double R = Line.Pos;

        Accel aX = Rail.NetAcc + Line.NetAcc*μ2*Beta.Pos;
        Accel aY = Cart.NetAcc - Line.NetAcc*μ1*Alfa.Pos;
        Accel aA =  (Cart.Acc - g*Alfa.Pos - 2*Alfa.Vel*Line.Vel) / R;
        Accel aB = -(Rail.Acc + g*Beta.Pos + 2*Beta.Vel*Line.Vel) / R;
        Accel aR = g - Line.NetAcc;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
        Line.Update(aR, dt);
        if (Fwind == 0.0)
            Line.Pos = R;
    }

    CraneState BasicLinearModel::GetState() const
    {
        CraneState s { Rail.Pos, Cart.Pos, Line.Pos };
        s.PayloadX = s.RailOffset + s.LiftLine * Beta.Pos;
        s.PayloadY = s.CartOffset - s.LiftLine * Alfa.Pos;
        s.PayloadZ = -s.LiftLine;
        return s;
    }

    //////////////////////////////////////////////////////////////////////

    void NonLinearConstLine::Update(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double sA2 = sA*sA, sB2 = sB*sB, cA2 = cA*cA;
        double R = Line.Pos;
        // cable driven tension/friction coefficients:
        double μ1 = Model.Mpayload / Cart.Mass; // μ1 = Mc / Mw
        double μ2 = Model.Mpayload / Rail.Mass; // μ2 = Mc / (Mw + Ms)

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

    void NonLinearComplete::Update(double dt, Force Frail, Force Fcart, Force Fwind)
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
        // cable driven tension/friction coefficients:
        double μ1 = Model.Mpayload / Cart.Mass; // μ1 = Mc / Mw
        double μ2 = Model.Mpayload / Rail.Mass; // μ2 = Mc / (Mw + Ms)

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

    void NonLinearOriginal::Update(double dt, Force Frail, Force Fcart, Force Fwind)
    {
        Rail.UpdateForce(Frail);
        Cart.UpdateForce(Fcart);
        Line.UpdateForce(Fwind);

        double sA = sin(Alfa.Pos), cA = cos(Alfa.Pos);
        double sB = sin(Beta.Pos), cB = cos(Beta.Pos);
        double sA2 = sA*sA, sB2 = sB*sB;
        double vB2 = Beta.Vel*Beta.Vel;
        double R = Line.Pos;
        double G = g.Value;
        double VA = R*vB2*cA*sA - 2*Line.Vel*Alfa.Vel + G*cA*cB;
        double VB = 2*Beta.Vel*(R*Alfa.Vel*cA + Line.Vel*sA) + G*sB;
        double VR = R*vB2*sA2 + G*sA*cB + R*Alfa.Vel*Alfa.Vel;
        // cable driven tension/friction coefficients:
        double μ1 = Model.Mpayload / Cart.Mass; // μ1 = Mc / Mw
        double μ2 = Model.Mpayload / Rail.Mass; // μ2 = Mc / (Mw + Ms)

        Accel aX = Rail.NetAcc + Line.NetAcc*μ2*sA*sB;
        Accel aY = Cart.NetAcc + Line.NetAcc*μ1*cA;
        Accel aA = (Cart.NetAcc*sA - Rail.NetAcc*cA*sB
                    - Line.NetAcc*cA*sA*μ2*sB*sB + Line.NetAcc*cA*sA*μ1 + VA) / R;
        Accel aB = -(Rail.NetAcc*cB + g*sB + 2*R*cA*Alfa.Vel*Beta.Vel +
                       Line.NetAcc*cB*μ2*sA*sB + 2*sA*Line.Vel*Beta.Vel) / (R*sA);
        Accel aR = -Cart.NetAcc*cA - Rail.NetAcc*sA*sB
                    - μ2*sA*sB*sA*sB*Cart.NetAcc - Line.NetAcc*μ1
                    + Line.NetAcc*μ1*sA2 - Line.NetAcc + VR;

        Rail.Update(aX, dt);
        Cart.Update(aY, dt);
        Alfa.Update(aA, dt);
        Beta.Update(aB, dt);
        if (Fwind != 0.0)
            Line.Update(aR, dt);
    }

    CraneState NonLinearModel::GetState() const
    {
        CraneState s { Rail.Pos, Cart.Pos, Line.Pos };
        double A = Alfa.Pos, B = Beta.Pos;
        s.PayloadX = s.RailOffset + s.LiftLine * sin(A) * sin(B);
        s.PayloadY = s.CartOffset + s.LiftLine * cos(A);
        s.PayloadZ = -s.LiftLine * sin(A) * cos(B);
        return s;
    }

    //////////////////////////////////////////////////////////////////////
}
