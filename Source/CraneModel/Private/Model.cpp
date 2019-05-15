// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "Model.h"
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <sstream> // std::stringstream
#include <iomanip> // std::setprecision
#include <algorithm>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////

    Model::Model(const string& selectedModel)
    {
        Rail.CoeffStaticColoumb = 5.0;
        Cart.CoeffStaticColoumb = 7.5;
        Line.CoeffStaticColoumb = 10.0;

        Rail.CoeffKineticViscous = 100.0;
        Cart.CoeffKineticViscous = 82.0;
        Line.CoeffKineticViscous = 75.0;

        Line.FrictionDir = -1.0;

        AddModel(std::make_shared<BasicLinearModel>(*this));
        AddModel(std::make_shared<NonLinearConstLine>(*this));
        AddModel(std::make_shared<NonLinearComplete>(*this));
        AddModel(std::make_shared<NonLinearOriginal>(*this));

        SetCurrentModelByName(selectedModel);
    }

    void Model::Reset()
    {
        SimulationTimeSink = 0.0;
        TotalSimTime = 0.0;
        TotalUpdates = 0;

        Rail.Reset();
        Cart.Reset();
        Line.Reset();
        Alfa.Reset();
        Beta.Reset();

        Line.Pos = 0.5;

        Rail.VelMax = VelocityMax;
        Cart.VelMax = VelocityMax;
        Line.VelMax = VelocityMax;

        Rail.AccMax = AccelMax;
        Cart.AccMax = AccelMax;
        Line.AccMax = AccelMax;

        if (CurrentModel->Name() == "Linear")
        {
            Alfa.SetLimits(-0.05, +0.05);
            Beta.SetLimits(-0.05, +0.05);
        }
        else
        {
            // Invert the alfa angle as required by non-linear models
            Alfa.Pos = _90degs;
            // the crane cannot physically swing more than X degrees due to cart edges
            Alfa.SetLimits(_60degs, _180degs - _60degs);
            Beta.SetLimits(-_30degs, _30degs);
        }
    }

    void Model::SetCurrentModelByName(const string& modelName)
    {
        auto model = Models.find(modelName);
        if (model == Models.end())
            throw std::runtime_error("Could not find model with id='"+modelName+"'");

        if (CurrentModel != model->second) {
            CurrentModel = model->second;
            Reset();
        }
    }

    vector<string> Model::GetModelNames() const
    {
        vector<string> names;
        std::transform(Models.begin(), Models.end(), std::back_inserter(names),
                       [](const auto& kv){ return kv.first; });
        return names;
    }

    void Model::AddModel(shared_ptr<IModelImplementation> model, bool setAsCurrent)
    {
        Models[model->Name()] = model;
        if (setAsCurrent) {
            SetCurrentModelByName(model->Name());
        }
    }

    void Model::SetOutputCsv(const std::string & outCsvFile)
    {
        DbgCsv = std::make_unique<std::ofstream>(outCsvFile);
        if (!DbgCsv->is_open())
            throw std::runtime_error("Failed to create CSV file");
        std::locale mylocale("et_EE.UTF-8"); 
        DbgCsv->imbue(mylocale);
        *DbgCsv << "t; X; Y; R; Alfa; Beta; payX; payY; payZ\n";
        AppendStateToCsv();
    }
    
    void Model::AppendStateToCsv()
    {
        CraneState s = GetState();
        *DbgCsv << TotalSimTime << "; " << Rail.Pos << "; "
                << Cart.Pos   << "; " << Line.Pos   << "; "
                << s.Alfa     << "; " << s.Beta     << "; "
                << s.PayloadX << "; " << s.PayloadY << "; " << s.PayloadZ << "\n";
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
        CraneState s = GetState();
        std::wstringstream ss;
        format(ss, L"Model: %hs \n", CurrentModel->Name().c_str());
        format(ss, L" Mpayl %6.1fkg \n", Mpayload.Value);
        format(ss, L" payl %+6.2f, %+6.2f, %+6.2f \n",
                    s.PayloadX, s.PayloadY, s.PayloadZ);
        format(ss, L" pXYR %+6.2f, %+6.2f, %+6.2f \n",
                    s.RailOffset, s.CartOffset, s.LiftLine);
        format(ss, L" vXYR %+6.2f, %+6.2f, %+6.2f m/s \n",
                        Rail.Vel, Cart.Vel, Line.Vel);
        format(ss, L"  α  %+6.2f  vα %+6.2f rad/s  aα %+6.2f rad/s^2 \n",
                        Alfa.Pos, Alfa.Vel, Alfa.Acc.Value);
        format(ss, L"  β  %+6.2f  vβ %+6.2f rad/s  aβ %+6.2f rad/s^2 \n",
                        Beta.Pos, Beta.Vel, Beta.Acc.Value);
        auto formatComponent = [&](const char* which, const Component& c) {
            format(ss, L"  %hs a %+6.2f m/s², Fnet %+5.1f, Fapp %+5.1f, "
                       L"Fst %+5.1f, Fki %+5.1f \n",
                which, c.Acc.Value, c.Fnet.Value, c.Applied.Value,
                c.SFriction.Value, c.KFriction.Value);
        };
        formatComponent("Rail", Rail);
        formatComponent("Cart", Cart);
        formatComponent("Line", Line);
        format(ss, L"  iter# %5lld  dt %f  iters/update %.1f  iter/s %.0f\n",
            TotalUpdates, DbgFixedTimeStep, DbgAvgIterations, TotalUpdates/TotalSimTime);
        return ss.str();
    }

    //////////////////////////////////////////////////////////////////////

    CraneState Model::UpdateFixed(double fixedTime, double elapsedTime,
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
        ++TotalUpdates;
        TotalSimTime += fixedTime;

        Rail.Mass = Mrail+Mcart;
        Cart.Mass = Mcart;
        Line.Mass = Mpayload;
        CurrentModel->Update(fixedTime, Frail, Fcart, Fwind);

        if (DbgCsv) {
            AppendStateToCsv();
        }
    }

    //////////////////////////////////////////////////////////////////////
}
