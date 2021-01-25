#ifndef BA_H
#define BA_H

#include "ceres/ceres.h"
namespace bA
{
	struct CostFunctor {
		CostFunctor(double s, double h2s, double f):s_(s), h2s_(h2s), f_(f){}
		
		template <typename T>
		bool operator()(const T* const d, const T* const H2, T* residual) const{
			residual[0] = T(h2s_) - H2[0]*T(f_)/(d[0]+T(s_));
			return true;
		}
		private:
			const double s_;
			const double h2s_;
			const double f_;
	};
	std::vector<double> sumStep(const std::vector<double> steps);
	bool optimizeDAndH(double& d, double& H2, const std::vector<double> h2, const std::vector<double> steps, const double f);
	
	
}
#endif


