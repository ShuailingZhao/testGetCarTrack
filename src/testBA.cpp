#include "bA.h"



const char * usage = 
"\n"
"./testBA"
"\n";

static void help()
{
	std::cout << usage;
}
int main(int argc, char** argv)
{
//	if(argc<2)
//	{
//		help();
//		return 0;
//	}
	
	
	std::vector<double> steps;
	std::vector<double> h2s;
	steps.push_back(4.8226554029);
	steps.push_back(4.5434018418);
	steps.push_back(4.4439593127);
	steps.push_back(4.64733);
	
//	steps.push_back(4.8226554029+4.5434018418+4.4439593127+4.64733);
//	steps.push_back(4.5434018418+4.4439593127+4.64733);
//	steps.push_back(4.4439593127+4.64733);
//	steps.push_back(4.64733);
//	steps.push_back(0.0);
	
	h2s.push_back(57.2548);
	h2s.push_back(62.2809);
	h2s.push_back(68.3604);
	h2s.push_back(74.4402);
	h2s.push_back(81.5394);
	
	double d = 48.0;
	double H2 = 2.73;
	double focal = 1470.29;
	bA::optimizeDAndH(d, H2, h2s, steps, focal);
	std::cout<<"d: "<<d<<" H2: "<<H2<<std::endl;

        
	return 0;
}
