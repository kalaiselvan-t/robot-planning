#include <iostream>
#include <vector>
#include <cmath>

double round_up(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}

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

double sinct
(double t)
{
	double out;
	if
	(std::abs(t) < 0.002)
	{
		out = 1 - std::pow(t, 2.0/6.0) * (1 - std::pow(t,2.0/20.0));
	}
	else
	{
		out = std::sin(t) / t;
	}

	return out;
}


struct cat
{
	double a;
	double b;
	double c;
};

void test_struc
(cat *inp)
{
	inp->a = 1.0;
	inp->b = 2.0;
	inp->c = 3.0;
}

int main
()
{	
	//Create function pointers
	void (*fn1)(int, int);
	void (*fn2)(int, int);
	void (*fn3)(int, int);
	void (*fn4)(int, int);

	//Vector definiton
	std::vector<void (*)(int, int)> my_func;

	//Initialization
	fn1 = &add;
	fn2 = &sub;
	fn3 = &prod;
	fn4 = &divi;

	//Add to vector
	my_func.push_back(fn1);
	my_func.push_back(fn2);
	my_func.push_back(fn3);
	my_func.push_back(fn4);
	
	//my_func[0](2,3);

	for(int i = 0; i < my_func.size(); i++)
	{
		my_func[i](2,3);
	}

	//std::cout << "sinct :" << sinct(0.001) << "\n";
	

	cat c;

	test_struc(&c);

	std::cout << "A: " << c.a << "\n";
	std::cout << "B: " << c.b << "\n";
	std::cout << "C: " << c.c << "\n";

	double tt = 3.4987564;

	double result = round_up(tt, 4);
	std::cout << "round_up: " << result << std::endl;

	return 0;
}
