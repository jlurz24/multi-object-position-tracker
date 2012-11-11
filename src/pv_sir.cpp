/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2004,2006 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 *  Implements a Position and Velocity Filter with a Position observation.
 *  The motion model is the so called IOU Integrated Ornstein-Uhlenbeck Process Ref[1]
 *    Velocity is Brownian with a trend towards zero proportional to the velocity
 *    Position is just Velocity integrated.
 *  This model has a well defined velocity and the mean squared speed is parameterised. Also the velocity correlation is parameterised.
 *  
 * Two implementations are demonstrated
 *  1) A direct filter
 *  2) An indirect filter where the filter is performed on error and state is estimated indirectly
 * Reference
 * [1] "Bayesian Multiple Target Tracking" Lawrence D Stone, Carl A Barlow, Thomas L Corwin
 */

#include "SIRFlt.hpp"
#include "models.hpp"
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
	typedef SIR_kalman_scheme FilterScheme;

	// Square 
	template <class scalar>
	inline scalar sqr(scalar x)
	{
		return x*x;
	}

	// Constant Dimensions
	const unsigned NX = 6;			// Filter State dimension 	(Position X Y Z, Velocity X Y Z)

	// Filter Parameters
	// Prediction parameters for Integrated Ornstein-Uhlembeck Process
	const Float dt = 0.01;
	const Float V_NOISE = 0.1;	// Velocity noise, giving mean squared error bound
	const Float V_GAMMA = 1.;	// Velocity correlation, giving velocity change time constant
	// Filter's Initial state uncertainty: System state is unknown
	const Float i_P_NOISE = 1.;
	const Float i_V_NOISE = 0.001;
	// Noise on observing system state
	const Float OBS_INTERVAL = 0.10;
	const Float OBS_NOISE = 0.1;

}//namespace


class Boost_random : public SIR_random, public Bayesian_filter_test::Boost_random
/*
 * Random numbers for SIR from Boost
 */
{
public:
	using Bayesian_filter_test::Boost_random::normal;
	void normal (DenseVec& v)
	{
		Bayesian_filter_test::Boost_random::normal (v);
	}
	using Bayesian_filter_test::Boost_random::uniform_01;
	void uniform_01 (DenseVec& v)
	{
		Bayesian_filter_test::Boost_random::uniform_01 (v);
	}
};

/*
 * Prediction model
 *  Sample from a Linear prediction with additive noise model
 */
class PVpredict : public Sampled_LiInAd_predict_model
{
public:
	PVpredict(Boost_random& rnd);
};

PVpredict::PVpredict(Boost_random& rnd) : Sampled_LiInAd_predict_model(NX, 3, rnd)
{
	// Position Velocity dependence
	const Float Fvv = exp(-dt*V_GAMMA);
	// Px
	Fx(0,0) = 1.;
	Fx(0,1) = 0.;
	Fx(0,2) = 0.;
	Fx(0,3) = dt;
	Fx(0,4) = 0.;
	Fx(0,5) = 0.;
	
	// Py
	Fx(1,0) = 0.;
	Fx(1,1) = 1.;
	Fx(1,2) = 0.;
	Fx(1,3) = 0.;
	Fx(1,4) = dt;
	Fx(1,5) = 0.;

	// Pz
	Fx(2,0) = 0.;
	Fx(2,1) = 0.;
	Fx(2,2) = 1.;
	Fx(2,3) = 0.;
	Fx(2,4) = 0.;
	Fx(2,5) = dt;

	// Vx
	Fx(3,0) = 0.;
	Fx(3,1) = 0.;
	Fx(3,2) = 0.;
	Fx(3,3) = exp(-dt * V_GAMMA);
	Fx(3,4) = 0.;
	Fx(3,5) = 0.;
	
	// Vy
        Fx(4,0) = 0.;
        Fx(4,1) = 0.;
        Fx(4,2) = 0.;
	Fx(4,3) = 0.;
        Fx(4,4) = exp(-dt * V_GAMMA);
        Fx(4,5) = 0.;

	// Vz
	Fx(5,0) = 0.;
        Fx(5,1) = 0.;
        Fx(5,2) = 0.;
        Fx(5,3) = 0.;
        Fx(5,4) = 0.;
        Fx(5,5) = exp(-dt * V_GAMMA);

	// Setup constant noise model: G is identity
	// TODO: Not sure if this is right
	q[0] = q[1] = q[2] = dt*sqr((1-Fvv)*V_NOISE);

	G(0,0) = 0.;
	G(1,0) = 0.;
	G(2,0) = 0.;
	G(3,0) = 1.;
	G(4,0) = 1.;
	G(5,0) = 1.;
}


/*
 * Position Observation model
 *  Likelihood function of a Linear observation with additive uncorrelated noise model
 */
class PVobserve : public General_LiUnAd_observe_model
{
	mutable Vec z_pred;
public:
	PVobserve ();
	const Vec& h(const Vec& x) const
	{
		// Copy over position but not velocity
		// as velocity is not observed.
		z_pred[0] = x[0];
		z_pred[1] = x[1];
		z_pred[2] = x[2];
		return z_pred;
	};
};

PVobserve::PVobserve () :
	General_LiUnAd_observe_model(NX,3), z_pred(3)
{
	// Linear model
	Hx(0,0) = 1;
	Hx(0,1) = 1;
	Hx(0,2) = 1;
	Hx(0,3) = 0.;
	Hx(0,4) = 0.;
	Hx(0,5) = 0.;

	// Observation Noise variance
	// TODO: Not sure if this is right
	cout << "Zv: " << Zv << endl;
	Zv[0] = Zv[1] = Zv[2] = sqr(OBS_NOISE);
}


void initialise (Kalman_state_filter& kf, const Vec& initState)
/*
 * Initialise Kalman filter with an initial guess for the system state and fixed covariance
 */
{
	// Initialise state guess and covarince
	kf.X.clear();
	// Position x, y, z
	kf.X(0,0) = sqr(i_P_NOISE);
	kf.X(1,1) = sqr(i_P_NOISE);
	kf.X(2,2) = sqr(i_P_NOISE);

	// Velocity x, y, z
	kf.X(3,3) = sqr(i_V_NOISE);
	kf.X(4,4) = sqr(i_V_NOISE);
	kf.X(5,5) = sqr(i_V_NOISE);
	kf.init_kalman (initState, kf.X);
}


int main()
{
	// global setup
	std::cout.flags(std::ios::scientific); std::cout.precision(6);

	// a random number generator
	Boost_random rnd;
	
	// Setup the test filters
	Vec x_true (NX);

	// True State to be observed
	x_true[0] = 1000.;	// Px
	x_true[1] = 100.;	// Py
	x_true[2] = 10.;	// Pz
	x_true[3] = 1.0;	// Vx
	x_true[4] = 1.0;	// Vy
	x_true[5] = 1.0;	// Vz
 
	std::cout << "Position Velocity" << std::endl;
	std::cout << "True Initial  " << x_true << std::endl;

	// Construct Prediction and Observation model and filter
	// Give the filter an initial guess of the system state
	PVpredict linearPredict(rnd);
	PVobserve linearObserve;
	Vec x_guess(NX);
	x_guess[0] = 1000.;
	x_guess[1] = 100.;
	x_guess[2] = 10.;
	x_guess[3] = 1.0;
	x_guess[4] = 1.0;
	x_guess[5] = 1.0;

	std::cout << "Guess Initial " << x_guess << std::endl;

	// f1 Direct filter construct and initialize with initial state guess
	FilterScheme f1(NX, 10, rnd);
	initialise(f1, x_guess);

	// Iterate the filter with test observations
	Vec u(3), z_true(3), z(3), z_temp(1);
	Float time = 0.; Float obs_time = 0.;
	for (unsigned i = 0; i < 10; ++i)
	{
		// Predict true state using Normally distributed acceleration
		// This is a Guassian
		cout << "Predicting true state" << endl;
		x_true = linearPredict.f(x_true);
		rnd.normal(u);	// normally distributed mean 0., stdDev for stationary IOU
		x_true[3] += u[0] * sqr(V_NOISE) / (2*V_GAMMA);
		x_true[4] += u[1] * sqr(V_NOISE) / (2*V_GAMMA);
		x_true[5] += u[2] * sqr(V_NOISE) / (2*V_GAMMA);

		// Predict filter with known perturbation
		cout << "Known perturbation predict" << endl;
		f1.predict(linearPredict);
		cout << "Known perturbation predict complete" << endl;
		time += dt;

		// Observation time
		if (obs_time <= time)
		{
			cout << "obs_time < time" << endl;
			
			// True Observation
			z_true[0] = x_true[0];
			z_true[1] = x_true[1];
			z_true[2] = x_true[2];

			cout << "creating randoms" << endl;
			// Observation with additive noise
			// normally distributed mean z_true[0], stdDev OBS_NOISE
			rnd.normal(z_temp, z_true[0], OBS_NOISE);
			z[0] = z_temp[0];
			rnd.normal(z_temp, z_true[1], OBS_NOISE);
			z[1] = z_temp[0];
			rnd.normal(z_temp, z_true[2], OBS_NOISE);
			z[2] = z_temp[0];
			

			// Filter observation
			cout << "Observing" << endl;
			f1.observe(linearObserve, z);

			obs_time += OBS_INTERVAL;
		}
	}

	// Update the filter to state and covariance are available
	cout << "Updating" << endl;
	f1.update();

	// Print everything: filter state and covariance
	std::cout << "True     " << x_true << std::endl;
	std::cout << "Direct   " << f1.x << ',' << f1.X <<std::endl;
	return 0;
}
