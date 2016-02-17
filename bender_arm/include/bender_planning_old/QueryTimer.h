#ifndef _QUERY_TIMER_H_
#define _QUERY_TIMER_H_

#include <time.h>

class QueryTimer{
private: 
	struct timespec ts;
	time_t start_sec;
	long start_nsec;
	time_t stop_sec;
	long stop_nsec;
public:
	QueryTimer();
	int Start();
	int Stop();
	double Duracion();
	~QueryTimer();
};

#endif
