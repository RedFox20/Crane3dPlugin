// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#include "Model.h"
#include "CraneController.h"
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

// regular C++ std::wcout doesn't handle Windows UCS-2 wstrings
void print_unicode(const std::wstring& text)
{
    DWORD written;
    WriteConsoleW(GetStdHandle(STD_OUTPUT_HANDLE), text.c_str(), (DWORD)text.size(), &written, 0);
}

using namespace crane3d;

int main()
{
    Model model { "NonLinearComplete" };
    model.SetOutputCsv("NonLinear_Cross_12s_34N.csv");

    CraneController controller {&model};
    controller.SetDrivingForces(34_N, 30_N, 34_N);
    controller.SetWayPoints({
        {-0.3,  0.0,  0.5, 0},
        {-0.3,  0.0,  0.5, 1.0}, // wait

        {+0.3,  0.0,  0.5, 0},
        {+0.3,  0.0,  0.5, 1.0}, // wait

        { 0.0,  0.0,  0.5, 0},   // go to center
        { 0.0,  0.0,  0.5, 1.0}, // wait

        { 0.0, -0.35, 0.5, 0},
        { 0.0, -0.35, 0.5, 1.0},

        { 0.0, +0.35, 0.5, 0},
        { 0.0, +0.35, 0.5, 1.0},

        { 0.0,  0.0,  0.5, 0},   // go to center
        { 0.0,  0.0,  0.5, 1.0}, // wait
    });
    controller.Run(0.001, 12.0);

    //Force Frail = 0_N;   // force driving the rail
    //Force Fcart = 25_N; // force driving the cart
    //Force Fwind = 0_N;   // force winding the cable
    //ModelState state = model.UpdateFixed(0.001, 5.0, Frail, Fcart, Fwind);
    //state.Print();

    auto text = model.GetStateDebugText();
    print_unicode(text);

	system("pause");
}