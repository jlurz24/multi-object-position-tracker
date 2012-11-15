/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 * Example of using Bayesian Filter Class to solve a simple problem.
 *  The example implements a Position and Velocity Filter with a Position observation.
 *  The motion model is the so called IOU Integrated Ornstein-Uhlenbeck Process Ref[1]
 *    Velocity is Brownian with a trend towards zero proportional to the velocity
 *    Position is just Velocity integrated.
 *  This model has a well defined velocity and the mean squared speed is parameterised. Also
 *  the velocity correlation is parameterised.
 *  
 * Two implementations are demonstrated
 *  1) A direct filter
 *  2) An indirect filter where the filter is performed on error and state is estimated indirectly
 * Reference
 * [1] "Bayesian Multiple Target Tracking" Lawrence D Stone, Carl A Barlow, Thomas L Corwin
 */

#include "UDFlt.hpp"
#include "filters/indirect.hpp"
#include "random.hpp"
#include <cmath>
#include <iostream>
#include <boost/numeric/ublas/io.hpp>

namespace
{
	using namespace Bayesian_filter;
	using namespace Bayesian_filter_matrix;
	using namespace std;
	// Choose Filtering Scheme to use
	typedef UD_scheme FilterScheme;

	// Square 
	template <class scalar>
	inline scalar sqr(scalar x)
	{
		return x*x;
	}

	// Random numbers from Boost
	Bayesian_filter_test::Boost_random localRng;

	// Constant Dimensions
	const unsigned NX = 6;			// Filter State dimension 	(Position[X, Y, Z], Velocity[X, Y, Z])

	// Filter Parameters
	// Prediction parameters for Integrated Ornstein-Uhlembeck Process
	const Float dt = 0.01;
	const Float V_NOISE = 0.1;	// Velocity noise, giving mean squared error bound
	const Float V_GAMMA = 1.;	// Velocity correlation, giving velocity change time constant
	// Filter's Initial state uncertainty: System state is unknown
	const Float i_P_NOISE = 1000.;
	const Float i_V_NOISE = 10.;
	// Noise on observing system state
	const Float OBS_INTERVAL = 0.10;
	const Float OBS_NOISE = 0.001;

}//namespace

/*
 * Prediction model
 * Linear state predict model
 */
class PVpredict : public Linear_predict_model
{
public:
	PVpredict();
};

PVpredict::PVpredict() : Linear_predict_model(NX, 1)
{
	// Position Velocity dependence
	const Float Fvv = exp(-dt*V_GAMMA);
	// Px
	Fx(0,0) = 1.;
	Fx(0,3) = dt;
	
	// Py
        Fx(1,1) = 1.;
        Fx(1,4) = dt;

	// Pz
        Fx(2,2) = 1.;
        Fx(2,5) = dt;

	// Vx
	Fx(3,3) = Fvv;
	
	// Vy
        Fx(4,4) = Fvv;

        // Vz       
	Fx(5,5) = Fvv;
	
	// Setup constant noise model: G is identity
	q[0] = dt*sqr((1-Fvv)*V_NOISE);
	
	G(0,0) = 0.;
	G(1,0) = 0.;
	G(2,0) = 0.;
	G(3,0) = 1.;
	G(4,0) = 1.;
	G(5,0) = 1.;
}


/*
 * Position Observation model
 * Linear observation is additive uncorrelated model
 */
class PVobserve : public Linrz_uncorrelated_observe_model
{
	mutable Vec z_pred;
public:
	PVobserve ();
	const Vec& h(const Vec& x) const
	{
		z_pred[0] = x[0];
		z_pred[1] = x[1];
		z_pred[2] = x[2];
		return z_pred;
	};
};

PVobserve::PVobserve () :
	Linrz_uncorrelated_observe_model(NX,3), z_pred(3)
{
	// Linear model
	Hx(0,0) = 1.;
	Hx(0,1) = 1.;
	Hx(0,2) = 1.;
	Hx(0,3) = 0.;
	Hx(0,4) = 0.;
	Hx(0,5) = 0.;

	// Observation Noise variance
	Zv[0] = Zv[1] = Zv[2] = sqr(OBS_NOISE);
}


void initialise (Kalman_state_filter& kf, const Vec& initState)
/*
 * Initialise Kalman filter with an initial guess for the system state and fixed covariance
 */
{
	// Initialise state guess and covarince
	kf.X.clear();
	kf.X(0,0) = sqr(i_P_NOISE);
	kf.X(1,1) = sqr(i_P_NOISE);
	kf.X(2,2) = sqr(i_P_NOISE);
	kf.X(3,3) = sqr(i_V_NOISE);
	kf.X(4,4) = sqr(i_V_NOISE);
	kf.X(5,5) = sqr(i_V_NOISE);

	kf.init_kalman (initState, kf.X);
}


int main()
{
	// global setup
	std::cout.flags(std::ios::scientific); std::cout.precision(6);

	// Setup the test filters
	Vec x_true (NX);

	// True State to be observed
	x_true[0] = 1000.;	// Position
	x_true[1] = 1000.;
	x_true[2] = 1000.;
	x_true[3] = 1.0;	// Velocity
	x_true[4] = 1.0;
	x_true[5] = 1.0;
 
	std::cout << "Position Velocity" << std::endl;
	std::cout << "True Initial  " << x_true << std::endl;

	// Construct Prediction and Observation model and filter
	// Give the filter an initial guess of the system state
	PVpredict linearPredict;
	PVobserve linearObserve;
	Vec x_guess(NX);
	x_guess[0] = 900.;
	x_guess[1] = 900.;
	x_guess[2] = 900.;
	x_guess[3] = 1.5;
	x_guess[4] = 1.5;
	x_guess[5] = 1.5;
	std::cout << "Guess Initial " << x_guess << std::endl;

	// f1 Direct filter construct and initialize with initial state guess
	FilterScheme f1(NX,NX);
	initialise (f1, x_guess);

	// f2 Indirect filter construct and Initialize with initial state guess
	FilterScheme error_filter(NX,NX);
	Indirect_kalman_filter<FilterScheme> f2(error_filter);
	initialise (f2, x_guess);


	// Iterate the filter with test observations
	Vec u(3), z_true(3), z(3), z_temp(1);
	Float time = 0.; Float obs_time = 0.;
	for (unsigned i = 0; i < 100; ++i)
	{
		// Predict true state using Normally distributed acceleration
		// This is a Guassian
		x_true = linearPredict.f(x_true);
		localRng.normal(u); // normally distributed mean 0., stdDev for stationary IOU
		x_true[3] += u[0]* sqr(V_NOISE) / (2*V_GAMMA);
		x_true[4] += u[1]* sqr(V_NOISE) / (2*V_GAMMA);
		x_true[5] += u[2]* sqr(V_NOISE) / (2*V_GAMMA);
		
		// Predict filter with known perturbation
		f1.predict (linearPredict);
		f2.predict (linearPredict);
		time += dt;

		// Observation time
		if (obs_time <= time)
		{
			// True Observation
			z_true[0] = x_true[0];
			z_true[1] = x_true[1];
			z_true[2] = x_true[2];

			// Observation with additive noise
			// normally distributed mean z_true[0], stdDev OBS_NOISE.
			localRng.normal (z_temp, z_true[0], OBS_NOISE);
			z[0] = z_temp[0];

			localRng.normal (z_temp, z_true[1], OBS_NOISE); 
                        z[1] = z_temp[0];
			localRng.normal (z_temp, z_true[2], OBS_NOISE); 
                        z[2] = z_temp[0];

			// Filter observation
			f1.observe (linearObserve, z);
			f2.observe (linearObserve, z);

			obs_time += OBS_INTERVAL;
		}
	}

	// Update the filter to state and covariance are available
	f1.update ();
	f2.update ();

	// Print everything: filter state and covariance
	std::cout <<"True     " << x_true << std::endl;
	std::cout <<"Direct   " << f1.x << ',' << f1.X << std::endl;
	std::cout <<"Indirect " << f2.x << ',' << f2.X << std::endl;
	return 0;
}
