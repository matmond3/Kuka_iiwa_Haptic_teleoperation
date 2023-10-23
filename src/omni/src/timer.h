#pragma once

//#include <mutex>
#include <string>
#include <chrono>
using namespace std::chrono;

namespace mimlab {

	class Timer{

	public:
		Timer();
		void tick(double dt);
		void tick();
		void reset();

		void get_stats(double &mean_dt, double &sd_dt, double &max_dt);
		std::string get_stats();

	private:
		//std::mutex mutex;

		high_resolution_clock::time_point last_t;		

		double max_dt;
		double s;
		double ss;
		int N;
	
	}; // class Timer

} // namespace mimlab