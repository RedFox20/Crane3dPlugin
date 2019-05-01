// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#pragma once

namespace crane3d
{
    /**
     * Allows switching between different crane model dynamics
     */
    enum class ModelType
    {
        // The most basic and foolproof crane model
        Linear,
        
        // Non-linear model with constant pendulum length with 2 control forces.
        // LiftLine (Fline) is ignored
        NonLinearConstantLine,

        // Non-linear fully dynamic model with all 3 forces
        NonLinearComplete,
    };

    struct Vec3d
    {
        double X = 0.0, Y = 0.0, Z = 0.0;

        Vec3d operator+(const Vec3d& v) const { return { X + v.X, Y + v.Y, Z + v.Z }; }
        Vec3d operator-(const Vec3d& v) const { return { X - v.X, Y - v.Y, Z - v.Z }; }
        Vec3d operator*(const Vec3d& v) const { return { X * v.X, Y * v.Y, Z * v.Z }; }
        Vec3d operator/(const Vec3d& v) const { return { X / v.X, Y / v.Y, Z / v.Z }; }
    };

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
    public:
        /**
         * NOTE: These are the customization parameters of the model
         */
        // Which model to use? Linear is simple and foolproof		
        ModelType Type = ModelType::Linear;
        double Mpayload = 1.000; // Mc mass of the payload
        double Mcart    = 1.155; // Mw mass of the cart
        double Mrail    = 2.200; // Ms mass of the moving rail
        double G = 9.81; // gravity constant, 9.81m/s^2

        double RailFriction = 100.0; // Tx rail friction
        double CartFriction = 82.0;  // Ty cart friction
        double LineFriction = 75.0;  // Tr liftline friction 

        // cart, rail, line limits
        double RailLimitMin = -30.0;
        double RailLimitMax = +30.0;

        double CartLimitMin = -35.0;
        double CartLimitMax = +35.0;

        double LineLimitMin = 5.0;
        double LineLimitMax = 90.0;

    private:
        double Xw = 0.0; // distance of the rail with the cart from the center of the construction frame
        double Yw = 0.0; // distance of the cart from the center of the rail;
        double R = 0.5; // length of the lift-line
        double Alfa = 0.0; // α angle between y axis (cart moving left-right) and the lift-line
        double Beta = 0.0; // β angle between negative direction on the z axis and the projection
                           // of the lift-line onto the xz plane
        
        // only used for basic linear model
        double Δα = 0.0, Δα_vel = 0.0;
        double Δβ = 0.0, Δβ_vel = 0.0;

        // velocity time derivatives
        double Xw_vel = 0.0;
        double Yw_vel = 0.0;
        double R_vel = 0.0;
        double Alfa_vel = 0.0;
        double Beta_vel = 0.0;

        // x1..x10 as per 3DCrane mathematical model description
        double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10; // state
        double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10; // derived state
        double s5, s7, c5, c7; // sinα, sinβ, cosα, cosβ
        double u1, u2, u3; // acceleration forces of cart, rail, line
        double T1, T2, T3; // friction forces of cart, rail, line
        double N1, N2, N3; // net acceleration of cart, rail, line
        double μ1, μ2; // payload/cart ratio;  payload/railcart ratio

        // simulation time sink for running correct number of iterations every update
        double SimulationTime = 0.0;
        int SimulationCounter = 0; // for debugging

    public:

        Model();

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

        void PrepareBasicRelations(double Frail, double Fcart, double Fline);
        
        // ------------------
        
        void BasicLinearModel(double dt, double Frail, double Fcart);

        // ------------------

        void PrepareNonLinearState();
        void DeriveNonLinearOutput(double dt, bool deriveLiftLine);

        void NonLinearConstantPendulum(double dt, double Frail, double Fcart);
        void NonLinearCompleteModel(double dt, double Frail, double Fcart, double Fline);

        // ------------------

        void ApplyLimits();
        void DampenAllValues();
    };

}
