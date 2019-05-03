#include <iostream>
#include "Model.h"

int main()
{
	using namespace crane3d;

	double Frail = 0.0; // force driving the rail
	double Fcart = 0.0; // force along the rail
	double Fwind = 0.0; // force winding the cable

	Model model;
    model.Type = ModelType::Linear;

	ModelState state = model.UpdateFixed(0.01, 10.0, Frail, Fcart, Fwind);
	state.Print();

	system("pause");
}