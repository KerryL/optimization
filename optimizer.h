// File:  optimizer.h
// Date:  8/4/2016
// Auth:  K. Loux
// Desc:  Base class for optimization algorithms.

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

// Standard C++ headers
#include <functional>

// Eigen headers
#include <Eigen/Eigen>

class Optimizer
{
public:
	struct AdditionalArgs
	{
		virtual ~AdditionalArgs() = default;
	};

	typedef std::function<Eigen::VectorXd(const Eigen::VectorXd&, const AdditionalArgs*)> ObjectiveFunction;
	Optimizer(ObjectiveFunction objectiveFunction, const unsigned int& iterationLimit, const AdditionalArgs* args = nullptr);
	virtual ~Optimizer() = default;

	virtual Eigen::VectorXd Optimize() const = 0;

protected:
	const ObjectiveFunction objectiveFunction;
	const unsigned int& iterationLimit;
	const AdditionalArgs* args;
};

#endif// OPTIMIZER_H_
