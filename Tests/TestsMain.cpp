#include <iostream>
#include "Model.h"

int main()
{
	using namespace crane3d;

	double Frail = 0.0; // force driving the rail
	double Fcart = 0.0; // force along the rail
	double Fline = 0.0; // force driving the cable

	Model model;
	ModelState state = model.Update(60.0, Frail, Fcart, Fline);
	state.Print();

	system("pause");
}