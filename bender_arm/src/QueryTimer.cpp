#include "bender_planning_old/QueryTimer.h"
/*Duracion() Devuelve el tiempo en segundos con resolucion [ns], entre Start() y Stop()*/
QueryTimer::QueryTimer(){}
int QueryTimer::Start(){
	clock_gettime(CLOCK_REALTIME, &ts);
	start_sec=ts.tv_sec;
	start_nsec=ts.tv_nsec;
	return 1;
}
int QueryTimer::Stop(){
	clock_gettime(CLOCK_REALTIME, &ts);
	stop_sec=ts.tv_sec;
	stop_nsec=ts.tv_nsec;
	return 1;
}
double QueryTimer::Duracion(){ // En segundos pero con resolucion de ns
	//printf("duracion segundos: %d\n duracion nanosegundos: %d\n",stop_sec-start_sec, stop_nsec-start_nsec);
	return (double)(stop_sec - start_sec)+((double)(stop_nsec - start_nsec))/1000000000.0;
}
QueryTimer::~QueryTimer(){}
