// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include "CraneState.h"
#include "KinematicComponent.h"
#include <string>  // std::wstring

namespace crane3d
{
    using std::string;
    //////////////////////////////////////////////////////////////////////

    class Model;

    class IModelImplementation
    {
    protected:
        Model& Model;
        Component &Rail, &Cart, &Line, &Alfa, &Beta;
        Accel& g;
    public:
        IModelImplementation(crane3d::Model& model);

        /** @return Name of this model as an unique identifier */
        virtual string Name() const = 0;

        /**
         * Updates the model using deltaTime as the time step.
         * @param dt Fixed time step
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         */
        virtual void Update(double dt, Force Frail, Force Fcart, Force Fwind) = 0;

        /** @return Current observable state of the Crane */
        virtual CraneState GetState() const = 0;
    };


    //////////////////////////////////////////////////////////////////////


    // The most basic crane model with minimum pendulum movement
    class BasicLinearModel : public IModelImplementation
    {
    public:
        using IModelImplementation::IModelImplementation;
        string Name() const override { return "Linear"; }
        void Update(double dt, Force Frail, Force Fcart, Force Fwind) override;
        CraneState GetState() const override;
    };


    //////////////////////////////////////////////////////////////////////


    // Base abstract class for non-linear models
    class NonLinearModel : public IModelImplementation
    {
    public:
        using IModelImplementation::IModelImplementation;
        CraneState GetState() const override;
    };


    // Non-linear model with constant pendulum length with 2 control forces.
    // LiftLine (Fwind) is ignored
    class NonLinearConstLine : public NonLinearModel
    {
    public:
        using NonLinearModel::NonLinearModel;
        string Name() const override { return "NonLinearConstLine"; }
        void Update(double dt, Force Frail, Force Fcart, Force Fwind) override;
    };


    // Non-linear fully dynamic model with all 3 forces
    class NonLinearComplete : public NonLinearModel
    {
    public:
        using NonLinearModel::NonLinearModel;
        string Name() const override { return "NonLinearComplete"; }
        void Update(double dt, Force Frail, Force Fcart, Force Fwind) override;
    };


    // Original non-linear fully dynamic model with all 3 forces and refined friction formulae
    class NonLinearOriginal : public NonLinearModel
    {
    public:
        using NonLinearModel::NonLinearModel;
        string Name() const override { return "NonLinearOriginal"; }
        void Update(double dt, Force Frail, Force Fcart, Force Fwind) override;
    };


    //////////////////////////////////////////////////////////////////////
}
