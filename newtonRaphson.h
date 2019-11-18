// File:  newtonRaphson.h
// Date:  10/12/2018
// Auth:  K. Loux
// Desc:  Optimizer using Newton-Raphson method.

#ifndef NEWTON_RAPHSON_H_
#define NEWTON_RAPHSON_H_

// Local headers
#include "optimizer.h"

// Standard C++ headers
#include <cassert>

template <int paramCount, int trueParamCount = paramCount>
class NewtonRaphson : public Optimizer
{
public:
	NewtonRaphson(ObjectiveFunction objectiveFunction,
		const unsigned int& iterationLimit, const AdditionalArgs* args = nullptr);

	Eigen::VectorXd Optimize() const override;

	typedef Eigen::Matrix<double, trueParamCount, 1> PointVec;
	typedef Eigen::Matrix<double, paramCount, 1> IntermediatePointVec;

	inline void SetTolerance(const double& t) { assert(t > 0.0); tolerance = t; }
	inline void SetMinAlpha(const double& a) { assert(a > 0.0); minAlpha = a; }
	inline void SetMinDelta(const double& d) { assert(d > 0.0); minDelta = d; }
	inline void SetEpsilon(const double& e) { assert(e > 0.0); epsilon = e; }
	inline void SetStepFactor(const double& f) { assert(f > 0.0 && f < 1.0); stepFactor = f; }

	inline void SetInitialGuess(const IntermediatePointVec& g) { initialGuess = g; }

	typedef IntermediatePointVec (*GuessUpdateFunction)(const IntermediatePointVec& v, const PointVec& delta);
	typedef bool (*GuessValidFunction)(const IntermediatePointVec& v, const AdditionalArgs* args);

	inline void SetGuessUpdateFunc(GuessUpdateFunction func) { guessUpdateFunc = func; }
	inline void SetGuessValidFunc(GuessValidFunction func) { guessValidFunc = func; }

private:
	double tolerance = 1.0e-6;
	double minAlpha = 1.0e-6;
	double minDelta = 1.0e-4;
	double epsilon = 1.0e-4;
	double stepFactor = 0.8;

	IntermediatePointVec initialGuess;

	static bool DefaultGuessValidFunction(const IntermediatePointVec&, const AdditionalArgs*) { return true; }
	GuessValidFunction guessValidFunc = &DefaultGuessValidFunction;

	template <int pc = paramCount, int tpc = trueParamCount>
	static typename std::enable_if<pc == tpc, IntermediatePointVec>::type DefaultGuessUpdateFunction(const IntermediatePointVec& v, const PointVec& delta) { return v + delta; }

	template <int pc = paramCount, int tpc = trueParamCount>
	static typename std::enable_if<pc != tpc, IntermediatePointVec>::type DefaultGuessUpdateFunction(const IntermediatePointVec& v, const PointVec&) { assert(false); return v; }

	GuessUpdateFunction guessUpdateFunc = &DefaultGuessUpdateFunction;

	bool AdjustStepIfDeltaIsLarge(PointVec& delta, double& lastDeltaLength) const;
};

//==========================================================================
// Class:			NewtonRaphson
// Function:		NewtonRaphson
//
// Description:		Constructor for NewtonRaphson class.
//
// Input Arguments:
//		objectiveFunction	= ObjectiveFunction
//		iterationLimit		= const unsigned int&
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
template <int paramCount, int trueParamCount>
NewtonRaphson<paramCount, trueParamCount>::NewtonRaphson(ObjectiveFunction objectiveFunction,
	const unsigned int& iterationLimit, const AdditionalArgs* args)
	: Optimizer(objectiveFunction, iterationLimit, args)
{
}
//==========================================================================
// Class:			NewtonRaphson
// Function:		Optimize
//
// Description:		Sovles for and returns arguments for the objective
//					function that optimize the result.
//
// Input Arguments:
//		None
//
// Output Arguments:
//		None
//
// Return Value:
//		Eigen::VectorXd
//
//==========================================================================
template <int paramCount, int trueParamCount>
Eigen::VectorXd NewtonRaphson<paramCount, trueParamCount>::Optimize() const
{
	assert(initialGuess.size() > 0);

	PointVec delta;
	double lastDeltaLength(0.0);
	IntermediatePointVec guess(initialGuess);
	for (unsigned int i = 0; i < iterationLimit; ++i)
	{
		while (!guessValidFunc(guess, args))
		{
			delta *= -0.5;
			if (delta.norm() < minDelta)
				break;
			guess = guessUpdateFunc(guess, delta);
		}

		const PointVec error(objectiveFunction(guess, args));
		if (error.norm() < tolerance)
			break;

		Eigen::Matrix<double, trueParamCount, trueParamCount> jacobian;
		for (int j = 0; j < jacobian.rows(); ++j)
		{
			PointVec tempDelta;
			tempDelta.setZero();
			tempDelta(j) = epsilon;
			const auto tempGuess(guessUpdateFunc(guess, tempDelta));
			auto tempError(objectiveFunction(tempGuess, args));
			jacobian.col(j) = (tempError - error) / epsilon;
		}

		delta = -jacobian.colPivHouseholderQr().solve(error);
		if (i > 0 && !AdjustStepIfDeltaIsLarge(delta, lastDeltaLength))
			break;
		lastDeltaLength = delta.norm();

		guess = guessUpdateFunc(guess, delta);
	}

	return guess;
}

template <int paramCount, int trueParamCount>
bool NewtonRaphson<paramCount, trueParamCount>::AdjustStepIfDeltaIsLarge(PointVec& delta, double& lastDeltaLength) const
{
	// If the step size is not getting smaller, make some adjustments (prevents large jumps due to numerical instability)
	if (delta.norm() > lastDeltaLength)
	{
		const double alpha(lastDeltaLength / delta.norm() * stepFactor);
		if (alpha < minAlpha)
			return false;
		delta *= alpha;
	}

	return true;
}

#endif// NEWTON_RAPHSON_H_
