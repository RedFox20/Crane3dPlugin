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

static void assert_expr(bool result, const char* expr, const char* message)
{
    if (result) fprintf(stdout, " [PASSED] %s\n", message);
    else        fprintf(stderr, " [ !!! FAILED !!! ]  (%s)  %s\n", expr, message);
}
#define Assert(expr, message) assert_expr(expr, #expr, message)

using namespace crane3d;

int main()
{

    Model model { "NonLinearOriginal" };
    //model.SetOutputCsv("NonLinear_Cross_6s_25N.csv");

    CraneController controller {&model};
    controller.SetDrivingForces(25_N, 25_N, 0_N);
    controller.SetWayPoints({
        {1.0, -0.30, 0.0},
        {1.0, +0.30, 0.0},
        {1.0, 0.0, 0.0}, // center
        {1.0, 0.0, -0.35},
        {1.0, 0.0, +0.35},
        {1.0, 0.0, 0.0}, // center
    });
    controller.Run(6.0, 0.001);

    //Force Frail = 0_N;   // force driving the rail
    //Force Fcart = 25_N; // force driving the cart
    //Force Fwind = 0_N;   // force winding the cable
    //ModelState state = model.UpdateFixed(0.001, 5.0, Frail, Fcart, Fwind);
    //state.Print();
    //auto text = model.GetStateDebugText();
    //print_unicode(text);

	system("pause");
}