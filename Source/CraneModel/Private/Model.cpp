// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#include "Model.h"
#include <cmath>
#include <cstdio>

namespace crane3d
{
	//////////////////////////////////////////////////////////////////////

	static constexpr double Mpayload = 1.000; // Mc mass of the payload
	static constexpr double Mcart = 1.155; // Mw mass of the cart
	static constexpr double Mrail = 2.200; // Ms mass of the moving rail
	static constexpr double Mrailcart = Mcart + Mrail;    // Mcart + Mrail    (Mw + Ms)
	static constexpr double Mcartpayload = Mcart + Mpayload; // Mcart + Mpayload (Mw + Mc)

	// Friction forces
	static constexpr double Tx = 100.0; // rail friction
	static constexpr double Ty = 82.0; // cart friction
	static constexpr double Tr = 75.0; // liftline friction 

	// time between each step of the simulation
	static constexpr double SimulationStep = 0.001;

	//////////////////////////////////////////////////////////////////////
	// Basice relations

	// μ1 = Mc / Mw
	static constexpr double PayloadCartRatio = Mpayload / Mcart;
	// μ2 = Mc / (Mw + Ms)
	static constexpr double PayloadRailCartRatio = Mpayload / Mrailcart;

	// T1 = Ty / Mw
	static constexpr double CartFrictionAccel = Ty / Mcart;
	// T2 = Tx / (Mw + Mc)
	static constexpr double RailFrictionAccel = Tx / Mcartpayload;
	// T3 = Tr / Mc
	static constexpr double LineFrictionAccel = Tr / Mpayload;

	// u1 = Fy / Mw
	// u1 = Fcart / Mcart
	static double CartAccel(double Fcart) { return Fcart / Mcart; }
	// u2 = Fx / (Mw + Mc)
	// u2 = Frail / Mcartpayload
	static double RailAccel(double Frail) { return Frail / Mcartpayload; }
	// u3 = Fr / Mc
	// u3 = Fline / Mpayload 
	static double LineAccel(double Fline) { return Fline / Mpayload; }

	// N1 = u1 - T1
	static double CartNetAccel(double Fcart) { return CartAccel(Fcart) - CartFrictionAccel; }
	// N2 = u2 - T2
	static double RailNetAccel(double Frail) { return RailAccel(Frail) - RailFrictionAccel; }
	// N3 = u3 - T3
	static double LineNetAccel(double Fline) { return LineAccel(Fline) - LineFrictionAccel; }

	// s = S / Mc
	static double LiftReactionAccel(double Sline) { return Sline / Mpayload; }

	constexpr double sign(double x)
	{
		if (x > 0) return +1.0;
		if (x < 0) return -1.0;
		return 0.0;
	}

	constexpr double VelocityDerivative(double x, double vel, double deltaTime)
	{
		return x + vel * deltaTime;
	}

	// dampen values that are very close to 0.0
	constexpr double Dampen(double x)
	{
		return abs(x) < 0.0000001 ? 0.0 : x;
	}

	//////////////////////////////////////////////////////////////////////

	ModelState Model::GetState() const
	{
		ModelState s;
		s.Alfa = Alfa;
		s.Beta = Beta;
		s.RailOffset = Xw;
		s.CartOffset = Yw;
		s.LiftLine = R;

		// Calculate payload position from Xw,Yw,α,β,R
		// Formula given by 3DCrane mathematical model description
		s.PayloadX = Xw + R * sin(Alfa) * sin(Beta);
		s.PayloadY = Yw + R * cos(Alfa);
		s.PayloadZ = -R * sin(Alfa) * cos(Beta);
		return s;
	}

	ModelState Model::Update(double deltaTime, double Frail, double Fcart, double Fline)
	{
		SimulationTime += deltaTime;

		int iterations = static_cast<int>(SimulationTime / SimulationStep);
		SimulationTime -= floor(SimulationTime);

		for (int i = 0; i < iterations; ++i)
		{
			//NonLinearConstantPendulum(Frail, Fcart);
			CompleteNonLinearModel(Frail, Fcart, Fline);
		}

		// and finally, return the observable current state
		return GetState();
	}

	void Model::NonLinearConstantPendulum(double Frail, double Fcart)
	{
		// x1..x8 as per 3DCrane mathematical model description
		double x1 = Yw;
		double x2 = Yw_vel; // velocity derivative of Yw
		double x3 = Xw;
		double x4 = Xw_vel;
		double x5 = (3.1415926 / 2) - Alfa;
		double x6 = Alfa_vel;
		double x7 = Beta;
		double x8 = Beta_vel;

		// NOTE: variable names left exactly the same as the original math. model descr.
		constexpr double g = 9.81; // gravity constant, 9.81m/s^2
		double s5 = sin(x5);
		double s7 = sin(x7);
		double c5 = cos(x5);
		double c7 = cos(x7);

		double N1 = CartNetAccel(Fcart);
		double N2 = RailNetAccel(Frail);
		double μ1 = PayloadCartRatio;
		double μ2 = PayloadRailCartRatio;

		double A = 1 + μ1 * c5*c5 + μ2 * s5*s5*s7*s7;
		double B = 1 + μ1;
		double V1 = μ1*R*c5*(x6*x6 + s5*s5*x8*x8) + μ1*g*c5*s5*c7;
		double V2 = μ2*R*s7*(s5*s5*s5*x8*x8 + s5*x6*x6) + μ2*g*s5*s5*c7*s7;
		double V3 = B*g*c5*c7 + B*R*c5*s5*x8*x8 + (μ1 - μ2*s7*s7)*R*c5*s5*x6*x6;
		double V4 = μ2*R*s5*c7*s7*(x6*x6 + s5*s5*x8*x8) + (B + (μ2 - μ1)*s5*s5)*g*s7 + 2*R*A*c5*x6*x8;

		// derived x1..x8 as per 3DCrane mathematical model description
		double d1 = x2;
		double d2 = ((1 + μ2*s5*s5*s7*s7)*N1 - μ1*c5*s5*s7*N2 + V1) / A;
		double d3 = x4;
		double d4 = (-μ2*c5*s5*s7*N1 + (1 + μ1*c5*c5)*N2 + V2) / A;
		double d5 = x6;
		double d6 = ((1 + μ2*s7*s7)*s5*N1 - B*c5*s7*N2 + V3) / (R*A);
		double d7 = x8;
		double d8 = 0.0; // if s5 is 0.0, then there is no Beta velocity
		if (abs(s5) > 0.000001) {
			d8 = (μ2*c5*s5*c7*s7*N1 - (B - μ1 * s5*s5)*c7*N2 - V4) / (R*A*s5);
		}

		// map the derived state to the current internal state:
		double dt = SimulationStep;
		Yw += dt * d1;
		Xw += dt * d3;
		Alfa += dt * ((3.1415926 / 2) - d5);
		Beta += dt * d7;
		Yw_vel += d2;
		Xw_vel = d4;
		Alfa_vel = d6;
		Beta_vel = d8;

		DampenAllValues(); // this dampening prevents INFINITY or NAN errors
	}

	void Model::CompleteNonLinearModel(double Frail, double Fcart, double Fline)
	{
		// x1..x10 as per 3DCrane mathematical model description
		double x1 = Yw;
		double x2 = Yw_vel; // velocity derivative of Yw
		double x3 = Xw;
		double x4 = Xw_vel;
		double x5 = (3.1415926 / 2) - Alfa;
		double x6 = Alfa_vel;
		double x7 = Beta;
		double x8 = Beta_vel;
		double x9 = R;
		double x10 = R_vel;

		// NOTE: variable names left exactly the same as the original math. model descr.
		constexpr double g = 9.81; // gravity constant, 9.81m/s^2
		double s5 = sin(x5);
		double s7 = sin(x7);
		double c5 = cos(x5);
		double c7 = cos(x7);
		double V5 = c5 * s5*x8*x8*x9 - 2 * x10*x6 + g * c5*c7;
		double V6 = 2 * x8*(c5*x6*x9 + s5 * x10) + g * s7;
		double V7 = s5 * s5*x8*x8*x9 + g * s5*c7 + x6 * x6*x9;

		double N1 = CartNetAccel(Fcart);
		double N2 = RailNetAccel(Frail);
		double N3 = LineNetAccel(Fline);
		double μ1 = PayloadCartRatio;
		double μ2 = PayloadRailCartRatio;

		// derived x1..x10 as per 3DCrane mathematical model description
		double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10;
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
		double un0 = Frail / Mcart;
		double un1 = Fcart / (Mcart + Mpayload);
		double un2 = Fline / Mpayload;
		double Tsx = 5 / (Mcart + Mrail);
		double Tsy = 7.5 / Mcart;
		double Tsz = 10 / Mpayload;
		double T1 = CartFrictionAccel;
		double T2 = RailFrictionAccel;
		double T3 = LineFrictionAccel;
		double mi1 = Mpayload / Mcart;
		double mi2 = Mpayload / (Mcart + Mrail);
		d1 = x2;
		d2 = un0 - x2 * T1 - Tsy * sign(x2) + mi1 * c5*un2 - mi1 * c5*(-x10 * T3 - Tsz * sign(x10));
		d3 = x4;
		d4 = un1 - x4 * T2 - Tsx * sign(x4) + mi2 * s5*s7*un2 - mi2 * s5*s7*(-x10 * T3 - Tsz * sign(x10));
		d5 = x6;
		d6 = (-(x2 * T1 + Tsy * sign(x2))*s5 + un0 * s5 + c5*x9 * s5*x8 * x8 - c5*un1 * s7 + c5*c7*g - c5*mi2*s5*s7*s7*un2 + c5*mi2*s5*s7*s7*(-x10 * T3 - Tsz * sign(x10)) + c5*(x4 * T2 + Tsx * sign(x4))*s7 + c5*mi1*un2 * s5 - c5*mi1*(-x10 * T3 - Tsz * sign(x10))*s5 - 2 * x10 * x6) / x9;
		d7 = x8;
		d8 = -(c7*un1 + g * s7 + 2 * x9 * c5*x6 * x8 + c7*mi2*s5*s7*un2 - c7*mi2*s5*s7*(-x10 * T3 - Tsz * sign(x10)) + 2 * x10 * s5*x8 - c7*(x4 * T2 + Tsx * sign(x4))) / (x9 * s5);
		d9 = x10;
		d10 = c5*(x2 * T1 + Tsy * sign(x2)) - c5*un0 + x9 * s5*s5*x8 * x8 - un1 * s5*s7 + s5*c7*g - mi2 * s5*s5*s7*s7*un2 + mi2 * s5*s5*s7*s7*(-x10 * T3 - Tsz * sign(x10)) + (x4 * T2 + Tsx * sign(x4))*s5*s7 - mi1 * un2 + mi1 * un2 * s5*s5 + mi1 * (-x10 * T3 - Tsz * sign(x10)) - mi1 * (-x10 * T3 - Tsz * sign(x10))*s5*s5 + x9 * x6 * x6 - un2 - x10 * T3 - Tsz * sign(x10);

		// map the derived state to the current internal state:
		double dt = SimulationStep;
		Yw   += dt * d1;
		Xw   += dt * d3;
		Alfa += dt * (d5);
		Beta += dt * d7;
		R += dt * d9; // TODO: how to apply outputs?
		Yw_vel += dt * d2;
		Xw_vel += dt * d4;
		Alfa_vel += dt * d6;
		Beta_vel += dt * d8;
		R_vel += dt * d10;

		//DampenAllValues(); // this dampening prevents INFINITY or NAN errors
	}

	void Model::DampenAllValues()
	{
		Yw = Dampen(Yw);
		Xw = Dampen(Xw);
		Alfa = Dampen(Alfa);
		Beta = Dampen(Beta);
		R = Dampen(R);

		Yw_vel = Dampen(Yw_vel);
		Xw_vel = Dampen(Xw_vel);
		Alfa_vel = Dampen(Alfa_vel);
		Beta_vel = Dampen(Beta_vel);
		R_vel = Dampen(R_vel);
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
