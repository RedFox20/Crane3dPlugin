// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "KinematicComponent.h"
#include <string> // std::wstring

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    /**
     * Allows switching between different crane model dynamics
     */
    enum class ModelType
    {
        // The most basic and foolproof crane model
        Linear,

        // Non-linear model with constant pendulum length with 2 control forces.
        // LiftLine (Fwind) is ignored
        NonLinearConstLine,


        // Non-linear fully dynamic model with all 3 forces
        NonLinearComplete,

        // Original non-linear fully dynamic model with all 3 forces and refined friction formulae
        NonLinearOriginal,
    };

    //////////////////////////////////////////////////////////////////////

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

    //////////////////////////////////////////////////////////////////////

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
        Mass Mpayload = 1.000_kg; // Mc mass of the payload
        Mass Mcart    = 1.155_kg; // Mw mass of the cart
        Mass Mrail    = 2.200_kg; // Ms mass of the moving rail
        double G { 9.81 };  // gravity constant, 9.81m/s^2
        Accel g = 9.81_ms2; // gravity constant, 9.81m/s^2

        // Rail component
        //   describes distance of the rail with the cart from the center of the construction frame
        Component Rail { -0.30, +0.30 };

        // Cart component
        //   describes distance of the cart from the center of the rail;
        Component Cart { -0.35, +0.35 };

        // Line component
        //   describes the length of the lift-line
        Component Line { +0.05, +0.90 };

        // Alfa component
        //   describes α angle between y axis (cart moving left-right) and the lift-line
        Component Alfa { -0.05, +0.05 }; // Alfa component

        // Beta component
        //   describes β angle between negative direction on the z axis and the projection
        //   of the lift-line onto the xz plane
        Component Beta { -0.05, +0.05 };

        double RailFriction = 100.0; // Tx rail friction
        double CartFriction = 82.0;  // Ty cart friction
        double WindingFriction = 75.0;  // Tr liftline winding friction 

    private:

        //double X = 0.0; // distance of the rail with the cart from the center of the construction frame
        //double Y = 0.0; // distance of the cart from the center of the rail;
        //double R = 0.5; // length of the lift-line
        //double Alfa = 0.0; // α angle between y axis (cart moving left-right) and the lift-line
        //double Beta = 0.0; // β angle between negative direction on the z axis and the projection
        //                   // of the lift-line onto the xz plane

        //// velocity time derivatives
        //double X_vel = 0.0; // rail X velocity
        //double Y_vel = 0.0; // cart Y velocity
        //double R_vel = 0.0; // payload Z velocity
        //double Alfa_vel = 0.0;
        //double Beta_vel = 0.0;

        //// x1..x10 as per 3DCrane mathematical model description
        //double CartNetAcc, RailNetAcc, WindNetAcc; // net acceleration of cart, rail, wind

        //double ADrcart, ADrrail, ADrwind; // driving accel of cart, rail, wind
        //double AFrcart, AFrrail, AFrwind; // T1,T2,T3 friction accel of cart, rail, wind
        double μ1, μ2; // coefficient of friction: payload/cart ratio;  payload/railcart ratio

        // simulation time sink for running correct number of iterations every update
        double SimulationTimeSink = 0.0;
        int64_t DiscreteStepCounter = 0;

        // for debugging:
        double DbgFixedTimeStep = 0.0;
        double DbgAvgIterations = 1.0; // for debugging

    public:

        Model();

        /**
         * Updates the model using a fixed time step
         * @param fixedTime Size of the fixed time step. For example 0.01
         * @param deltaTime Time since last update
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        ModelState UpdateFixed(double fixedTime, double deltaTime, Force Frail, Force Fcart, Force Fwind);

        /**
         * Updates the model using deltaTime as the time step. This can be unstable if deltaTime varies.
         * @param deltaTime Time since last update
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        ModelState Update(double deltaTime, Force Frail, Force Fcart, Force Fwind);

        /**
         * @return Current state of the crane:
         *  distance of the rail, cart, length of lift-line and swing angles of the payload
         */
        ModelState GetState() const;

        std::wstring GetStateDebugText() const;

    private:


        void PrepareBasicRelations(Force Frail, Force Fcart, Force Fwind);
        
        // ------------------
        
        void BasicLinearModel(double dt, Force Frail, Force Fcart, Force Fwind);

        // ------------------

        void NonLinearConstLine(double dt, Force Frail, Force Fcart, Force Fwind);
        void NonLinearCompleteModel(double dt, Force Frail, Force Fcart, Force Fwind);
        void NonLinearOriginalModel(double dt, Force Frail, Force Fcart, Force Fwind);

        // ------------------
    };

    //////////////////////////////////////////////////////////////////////
}
