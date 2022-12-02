#include <iostream>
#include <vector>
#include <cmath>
#include "test.h"

// int main
// ()
// {	
// 	//Create function pointers
// 	void (*fn1)(int, int);
// 	void (*fn2)(int, int);
// 	void (*fn3)(int, int);
// 	void (*fn4)(int, int);

// 	//Vector definiton
// 	std::vector<void (*)(int, int)> my_func;

// 	//Initialization
// 	fn1 = &add;
// 	fn2 = &sub;
// 	fn3 = &prod;
// 	fn4 = &divi;

// 	//Add to vector
// 	my_func.push_back(fn1);
// 	my_func.push_back(fn2);
// 	my_func.push_back(fn3);
// 	my_func.push_back(fn4);
	
// 	//my_func[0](2,3);

// 	for(int i = 0; i < my_func.size(); i++)
// 	{
// 		my_func[i](2,3);
// 	}

// 	//std::cout << "sinct :" << sinct(0.001) << "\n";
	

// 	cat c;

// 	test_struc(&c);

// 	std::cout << "A: " << c.a << "\n";
// 	std::cout << "B: " << c.b << "\n";
// 	std::cout << "C: " << c.c << "\n";

// 	double tt = 3.4987564;

// 	double result = round_up(tt, 4);
// 	std::cout << "round_up: " << result << std::endl;

// 	int aa;
// 	double bb;
// 	char cc;

// 	int jj = 9;

// 	int *jj_add = &jj;

// 	*jj_add = 10;

// 	pointer_test(aa, bb, cc);

// 	std::cout << jj_add << " " << *jj_add << " " << jj << std::endl;

// 	return 0;
// }

struct cat c;

int main()
{
	add(5,3);
	sub(5,3);
	prod(5,3);
	divi(5,3);
	// c.a = 1.0;
	// c.b = 2.0;
	// c.c = 3.0;
	test_struc(&c);
	std::cout << c.a << "," << c.b << "," << c.c << std::endl;
	return 1;
}
