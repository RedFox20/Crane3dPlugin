// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#pragma once

namespace crane3d
{
	/**
	 * Output state of the model
	 */
	struct ModelState
	{
		double Alfa = 0.0; // α pendulum measured alfa angle
		double Beta = 0.0; // β pendulum measured beta angle

		double RailOffset = 0.0; // Xw distance of the rail with the cart from the center of the construction frame
		double CartOffset = 0.0; // Yw distance of the cart from the center of the rail
		double LiftLine   = 0.0; // R lift-line length

		// Payload 3D coordinates
		double PayloadX = 0.0;
		double PayloadY = 0.0;
		double PayloadZ = 0.0;

		void Print() const;
	};

	// Coordinate system of the Crane model
	// X: outermost movement of the rail, considered as forward
	// Y: left-right movement of the cart
	// Z: up-down movement of the payload
	class Model
	{
		double Xw = 0.0; // distance of the rail with the cart from the center of the construction frame
		double Yw = 0.0; // distance of the cart from the center of the rail;
		double R = 0.5; // length of the lift-line
		double Alfa = 0.0; // α angle between y axis (cart moving left-right) and the lift-line
		double Beta = 0.0; // β angle between negative direction on the z axis and the projection
						   // of the lift-line onto the xz plane

		// velocity time derivatives
		double Xw_vel = 0.0;
		double Yw_vel = 0.0;
		double R_vel = 0.0;
		double Alfa_vel = 0.0;
		double Beta_vel = 0.0;

		// simulation time sink for running correct number of iterations every update
		double SimulationTime = 0.0;

		int SimulationCounter = 0; // for debugging

	public:

		Model() = default;

		/**
		 * @param deltaTime Time since last update
		 * @param Frail force driving the rail with cart (Fx)
		 * @param Fcart force driving the cart along the rail (Fy)
		 * @param Fline force controlling the length of the lift-line (Fr)
		 * @return New state of the crane model
		 */
		ModelState Update(double deltaTime, double Frail, double Fcart, double Fline);

		/**
		 * @return Current state of the crane:
		 *  distance of the rail, cart, length of lift-line and swing angles of the payload
		 */
		ModelState GetState() const;

	private:

		void NonLinearConstantPendulum(double Frail, double Fcart);

		void CompleteNonLinearModel(double Frail, double Fcart, double Fline);

		void DampenAllValues();
	};

}
