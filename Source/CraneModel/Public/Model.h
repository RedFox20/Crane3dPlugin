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
        // The most basic crane model with minimum pendulum movement
        Linear,

        // Non-linear fully dynamic model with all 3 forces
        NonLinearComplete,

        // Non-linear model with constant pendulum length with 2 control forces.
        // LiftLine (Fwind) is ignored
        NonLinearConstLine,

        // Original non-linear fully dynamic model with all 3 forces and refined friction formulae
        NonLinearOriginal,
    };

    //////////////////////////////////////////////////////////////////////

    /**
     * Output state of the model
     */
    struct ModelState
    {
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
        /** NOTE: These are the customization parameters of the model */
        Mass Mpayload = 1.000_kg; // Mc mass of the payload
        Mass Mcart    = 1.155_kg; // Mw mass of the cart
        Mass Mrail    = 2.200_kg; // Ms mass of the moving rail
        Accel g = 9.81_ms2; // gravity constant, 9.81m/s^2

        // Rail component
        //   describes distance of the rail from the center of the frame
        Component Rail { 0.0, -0.30, +0.30 };

        // Cart component
        //   describes distance of the cart from the center of the rail;
        Component Cart { 0.0, -0.35, +0.35 };

        // Line component
        //   describes the length of the lift-line
        Component Line { 0.5, +0.08, +0.90 };

        // Alfa component
        //   describes α angle between y axis (cart left-right) and the lift-line
        Component Alfa { 0.0, -0.05, +0.05 }; // Alfa component

        // Beta component
        //   describes β angle between negative direction on the z axis and
        //   the projection of the lift-line onto the xz plane
        Component Beta { 0.0, -0.05, +0.05 };

        // Maximum crane component velocity for Rail, Cart, Line
        double VelocityMax = 0.3; // m/s

        // Maximum crane component acceleration
        double AccelMax = 0.6; // m/s^2

    private:

        ModelType Type = ModelType::Linear;
        double μ1, μ2; // coefficient of friction: payload/cart ratio;  payload/railcart ratio

        // simulation time sink for running correct number of iterations every update
        double SimulationTimeSink = 0.0;
        int64_t DiscreteStepCounter = 0;

        // for debugging:
        double DbgFixedTimeStep = 0.0;
        double DbgAvgIterations = 1.0; // for debugging

    public:

        /**
         * Initialize model with a specific model type
         */
        Model(ModelType type = ModelType::Linear);

        /**
         * Resets all simulation components. Does not modify customization parameters. 
         */
        void Reset();

        /**
         * Sets the simulation type and Resets the simulation if the type changed.
         */
        void SetType(ModelType type);
        ModelType GetType() const { return Type; }


        /**
         * Updates the model using a fixed time step
         * @param fixedTime Size of the fixed time step. For example 0.01
         * @param elapsedTime Time since last update
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        ModelState UpdateFixed(double fixedTime, double elapsedTime, Force Frail, Force Fcart, Force Fwind);

        /**
         * Updates the model using deltaTime as the time step. This can be unstable if deltaTime varies.
         * @param deltaTimeStep Fixed time step
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        void Update(double deltaTimeStep, Force Frail, Force Fcart, Force Fwind);

        /**
         * @return Current state of the crane:
         *  distance of the rail, cart, length of lift-line and swing angles of the payload
         */
        ModelState GetState() const;

        std::wstring GetStateDebugText() const;

    private:

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
