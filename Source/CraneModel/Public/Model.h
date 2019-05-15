// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "KinematicComponent.h"
#include "ModelImplementation.h"
#include <vector>  // std::vector
#include <fstream> // std::ofstream
#include <memory>  // std::shared_ptr
#include <unordered_map> // std::unordered_map

namespace crane3d
{
    using std::vector;
    using std::shared_ptr;
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
        Component Rail { 0.0, -0.28, +0.28 };

        // Cart component
        //   describes distance of the cart from the center of the rail;
        Component Cart { 0.0, -0.39, +0.39 };

        // Line component
        //   describes the length of the lift-line
        Component Line { 0.5, +0.18, +0.70 };

        // Alfa component
        //   describes α angle between y axis (cart left-right) and the lift-line
        Component Alfa { 0.0, -0.05, +0.05 };

        // Beta component
        //   describes β angle between negative direction on the z axis and
        //   the projection of the lift-line onto the xz plane
        Component Beta { 0.0, -0.05, +0.05 };

        // Maximum Rail, Cart, Line component velocity
        double VelocityMax = 0.3; // m/s

        // Maximum Rail, Cart, Line component acceleration
        double AccelMax = 0.6; // m/s^2

    private:

        shared_ptr<IModelImplementation> CurrentModel;
        std::unordered_map<string, shared_ptr<IModelImplementation>> Models;

        double SimulationTimeSink = 0.0; // accumulator for running N iterations every update
        double TotalSimTime = 0.0; // total simulation time elapsed
        int64_t TotalUpdates = 0; // total number of discrete steps taken

        // for debugging:
        double DbgFixedTimeStep = 0.0;
        double DbgAvgIterations = 1.0;
        std::unique_ptr<std::ofstream> DbgCsv; // CSV output file stream

    public:

        /**
         * Initialize model with a default model type. Throws if default model doesn't exist.
         */
        Model(const string& selectedModel = "Linear");

        Model(Model&&)      = delete; // NoMove
        Model(const Model&) = delete; // NoCopy
        Model& operator=(Model&&)      = delete; // NoMove
        Model& operator=(const Model&) = delete; // NoCopy

        /**
         * Resets all simulation components. Does not modify customization parameters. 
         */
        void Reset();

        /**
         * Sets the simulation type and Resets the simulation if the type changed.
         * Throws if model with this name is not found.
         */
        void SetCurrentModelByName(const string& modelName);
        string GetCurrentModelName() const { return CurrentModel->Name(); }

        /** @return List of all supported model type names */
        vector<string> GetModelNames() const;

        /**
         * Register a new model implementation
         * @param setAsCurrent [false] If TRUE, sets the added model as CurrentModel
         */
        void AddModel(shared_ptr<IModelImplementation> model, bool setAsCurrent = false);

        /**
         * @return Current time state of the simulation
         */
        double GetSimulationTime() const { return TotalSimTime; }

        /**
         * Writes all simulation data points to a CSV file for later analysis
         * Columns exported:
         * t; X; Y; R; Alfa; Beta; pX; pY; pZ
         */
        void SetOutputCsv(const std::string& outCsvFile);

        /**
         * Updates the model using a fixed time step
         * @param fixedTime Size of the fixed time step. For example 0.01
         * @param elapsedTime Time since last update
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        CraneState UpdateFixed(double fixedTime, double elapsedTime,
                               Force Frail, Force Fcart, Force Fwind);

        /**
         * Updates the model using deltaTime as the time step.
         * This can be unstable if deltaTime varies.
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
        CraneState GetState() const { return CurrentModel->GetState(); }

        /**
         * @return A full multi-line debug string with all dynamic variables shown
         */
        std::wstring GetStateDebugText() const;

    private:
        void AppendStateToCsv();
    };

    //////////////////////////////////////////////////////////////////////
}
