#include "BayesFilter/unsFlt.hpp"
#include <iostream>
#include <boost/numeric/ublas/io.hpp>

using namespace std;
using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;


/*
 * Simple Prediction model
 */
class Simple_predict : public Linear_predict_model
{
public:
	Simple_predict() : Linear_predict_model(1,1)
	// Construct a constant model
	{
				// Stationary Prediction model (Identity)
		Fx(0,0) = 1.;
				// Constant Noise model with a large variance
		q[0] = 2.;
		G(0,0) = 1.;
	}
};

/*
 * Simple Observation model
 */
class Simple_observe : public Linear_uncorrelated_observe_model
{
public:
	Simple_observe () : Linear_uncorrelated_observe_model(1,1)
	// Construct a constant model
	{
				// Linear model
		Hx(0,0) = 1;
				// Constant Observation Noise model with variance of one
		Zv[0] = 1.;
	}
};


/*
 * Filter a simple example
 */
int main()
{
	// Global setup for test output
	cout.flags(ios::fixed); cout.precision(4);

	// Construct simple Prediction and Observation models
	Simple_predict my_predict;
	Simple_observe my_observe;

	// Use an 'unscented' filter scheme with one state
	Unscented_scheme my_filter(1);

	// Setup the initial state and covariance
	Vec x_init (1); SymMatrix X_init (1, 1);
	x_init[0] = 10.;		// Start at 10 with no uncertainty
	X_init(0,0) = 0.;
	my_filter.init_kalman (x_init, X_init);

	cout << "Initial  " << my_filter.x << my_filter.X << endl;

	// Predict the filter forward
	my_filter.predict (my_predict);
	my_filter.update();		// Update the filter, so state and covariance are available

	cout << "Predict  " << my_filter.x << my_filter.X << endl;

	// Make an observation
	Vec z(1);
	z[0] = 11.;				// Observe that we should be at 11
	my_filter.observe (my_observe, z);
	my_filter.update();		// Update the filter to state and covariance are available

	cout << "Filtered " << my_filter.x << my_filter.X << endl;
	return 0;
}

