#include "bA.h"
namespace bA{
//	template <typename T>
//	bool CostFunctor<T>::operator()(const T* const x, T* residual) const {
//		residual[0] = T(10.0) - x[0];
//		return true;
//	}
	std::vector<double> sumStep(const std::vector<double> steps)
	{
		std::vector<double> sumSteps;
		double temSum = 0.0;
		for(int i=0;i<steps.size();i++)
		{
			temSum += steps[steps.size()-1-i];
			sumSteps.push_back(temSum);
		}
		std::reverse(sumSteps.begin(), sumSteps.end());
		return sumSteps;
	}
	
	bool optimizeDAndH(double& d, double& H2, const std::vector<double> h2, const std::vector<double> steps, const double f)
	{
		if(1 != h2.size()-steps.size())
		{
			return false;
		}
		double initialD = d;
		double initialH2 = H2;
		// 建立Problem
		ceres::Problem problem;
		// 建立CostFunction（残差方程）
		double focal = f;
		std::vector<double> sumSteps = sumStep(steps);
		double currentFrameStep=0.0;
		sumSteps.push_back(currentFrameStep);
		
		for(int i=0;i<sumSteps.size();i++)
		{
			std::cout<<sumSteps[i]<<std::endl;
		}
		
		for(int i=0;i<sumSteps.size();i++)
		{
			ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<bA::CostFunctor, 1, 1, 1>(new bA::CostFunctor(sumSteps[i], h2[i], focal));
			problem.AddResidualBlock(cost_function, NULL, &d, &H2);

		}
		// 求解方程!
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = true;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

//		std::cout << summary.BriefReport() << "\n";
//		std::cout << "d : " << initialD
//		     << " -> " << d << "\n";
//		     
//		std::cout << "H2 : " << initialH2
//		     << " -> " << H2 << "\n";

		return true;

	}
}
