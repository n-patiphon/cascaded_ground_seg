#include <iostream>
#include <vector>
void print_vector(std::vector<int> in)
{
	for (int i=0; i<in.size(); i++)
	{
		std::cout << in[i] << std::endl;
	}
}
int main()
{
	std::vector<int> a(5, 100);
	std::vector<int> b(4, 300);
	
	std::cout << "Before inserted!!" << std::endl;
	print_vector(a);

	a.insert(a.end(), b.begin(), b.end());

	std::cout << "After inserted!!" << std::endl;
	print_vector(a);
	
}

