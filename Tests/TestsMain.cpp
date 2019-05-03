#include <iostream>
#include "Model.h"

int main()
{
	using namespace crane3d;

    Force Frail { 0.0 }; // force driving the rail
    Force Fcart { 0.0 }; // force along the rail
    Force Fwind { 0.0 }; // force winding the cable

	Model model;
    model.Type = ModelType::Linear;

	ModelState state = model.UpdateFixed(0.01, 10.0, Frail, Fcart, Fwind);
	state.Print();

	system("pause");
}