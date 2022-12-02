#include <iostream>
#include <vector>
#include <cmath>
#include "test.h"

// struct cat c;

// double round_up
// (double value, int decimal_places)
// {
//     const double multiplier = std::pow(10.0, decimal_places);
//     return std::ceil(value * multiplier) / multiplier;
// }

void add
(int a, int b)
{
	std::cout << "Addition: " << a + b << "\n";
}

void sub
(int a, int b)
{
	std::cout << "Subtraction: " << a - b << "\n";
}

void prod
(int a, int b)
{
	std::cout << "Product: " << a * b << "\n";
}

void divi
(int a, int b)
{
	std::cout << "Division: " << a / b << "\n";
}

// double sinct
// (double t)
// {
// 	double out;
// 	if
// 	(std::abs(t) < 0.002)
// 	{
// 		out = 1 - std::pow(t, 2.0/6.0) * (1 - std::pow(t,2.0/20.0));
// 	}
// 	else
// 	{
// 		out = std::sin(t) / t;
// 	}

// 	return out;
// }

void test_struc
(cat *inp)
{
	inp->a = 1.0;
	inp->b = 2.0;
	inp->c = 3.0;
}

void pointer_test
(int &a, double &b, char &c)
{
	a = 10;
	b = 3.0;
	c = 'k';
}
