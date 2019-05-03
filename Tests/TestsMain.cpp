#include <iostream>
#include "Model.h"
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

int main()
{
	using namespace crane3d;

    Force Frail { 50.0 }; // force driving the rail
    Force Fcart { 0.0 }; // force along the rail
    Force Fwind { 0.0 }; // force winding the cable

	Model model;
    model.Type = ModelType::Linear;

    Force Fnet = model.NetForce(-1_N, -5.0, 1_kg, 0.5, 0.25);

	ModelState state = model.UpdateFixed(0.01, 10.0, Frail, Fcart, Fwind);
	state.Print();

    auto text = model.GetStateDebugText();
    print_unicode(text);

	system("pause");
}