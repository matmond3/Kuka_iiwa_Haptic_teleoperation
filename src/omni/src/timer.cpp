#include <sstream> 
#include <iomanip>
#include <cmath>
#include "timer.h"
using namespace mimlab;

Timer::Timer() {
	this->reset();	
}

void Timer::tick(double dt) {
	{
		//std::scoped_lock
		//	lock(mutex);

		s += dt;
		ss += dt * dt;
		N++;

		if (dt > max_dt) {
			max_dt = dt;
		}
	}
}

void Timer::tick() {
	high_resolution_clock::time_point t = high_resolution_clock::now();
	if (last_t < t) {
		auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t - last_t);
		this->tick((double)dt.count());
	}
	last_t = t;
}

void Timer::reset() {
	{
		//std::scoped_lock
		//	lock(mutex);

		max_dt = 0.0;
		s = 0.0;
		ss = 0.0;
		N = 0;
		last_t = high_resolution_clock::now() + std::chrono::hours(48);
	}
}

void Timer::get_stats(double& mean_dt, double& sd_dt, double& max_dt) {
	if (N > 0) {
		{
			//std::scoped_lock
			//	lock(mutex);

			max_dt = this->max_dt;
			mean_dt = this->s / (double)this->N;
			sd_dt = sqrt((ss - pow(s, 2) / (double)N) / (double)N);
		}
	}
	else {
		mean_dt = sd_dt = max_dt = 0.0;
	}
}

std::string Timer::get_stats() {
	double mean_dt, sd_dt, max_dt;
	this->get_stats(mean_dt, sd_dt, max_dt);	
	std::ostringstream streamObj;		
	streamObj << std::fixed;	
	streamObj << std::setprecision(3);
	streamObj << "mean +/- sd = " << mean_dt << " +/- " << sd_dt << "; max = " << max_dt;

	return streamObj.str();
}