#pragma once

#include "utilities.h"
#include <math.h>
#include "Segment.h"
#include "SegmentFeatures.h"
#include <vector>
#include <limits>

#define DEBUG_OUTPUT 0

#define SEGMENT_DISTANCE_THRESHOLD (150)  // Maxima distancia entre puntos sucesivos de un segmento (en mm)
#define MIN_SEGMENT_LENGTH         (150)  // Minima longitud de segmento (en mm)
#define MAX_SEGMENT_LENGTH         (650)  // Maxima longitud de segmento (en mm)
#define MAX_SEGMENT_DISTANCE       (3800) // Maxima distancia del candidato al laser

#define MAX_TRACKING_FEATURES_DISTANCE    (2000) // Maxima distancia entre caracteristicas entre segmento candidato y trackeado para que candidato sea valido
#define MAX_DISPLACEMENT_BETWEEN_READINGS (500)  // Maximo desplazamientos de 50 Segm entre lecturas, sino es sospechoso

#define OCCLUSION_RISK_DISTANCE (500) // Si hay un segmento a esta distancia, se considera riesgo de oclusion

#define MIN_CYCLES_OF_RISK_BEFORE_OCCLUSION (5)  // Numero de ciclos que debe durar riesgo de oclusion para considerar que realmente hay que hacer manejo de oclusion
#define MAX_CYCLES_IN_OCCLUSION             (20)  // Numero maximo de ciclos de manejo de oclusion

#define MIN_SEGMENT_DATA_SIZE (10)        // Numero de lecturas minima que debe tener un segmento

#define MIN_ANGLE_RANGE (-90) // Inicio del rango de interes (segmentos fuera del rango son descartados)
#define MAX_ANGLE_RANGE (90)  // Fin del rango de interes

//#define RESTART_TRACKING_RADIUS (1000)
#define RECOVERY_RADIUS_INCREMENT (200)
#define MAX_RECOVERY_RADIUS       (1400)

enum {ST_CALIBRATING, ST_TRACKING};

enum TrackingStatus {
	TRACK_OK,
	TRACK_LOST,
	TRACK_OCCLUSION_RISK,
	TRACK_OCCLUSION,
	TRACK_RECOVER,
	TRACK_NOT_AVAILABLE
};

class PersonTracker
{
public:
	PersonTracker(int numData, float resolution);
	~PersonTracker(void);

	void processNewReading(long* data, int data_max);
	const std::vector<Segment>& getSegments();    // Obtiene lista de segmentos
	bool isCalibrationOk();                       // Pregunta si hay segmento detectado al medio para empezar a trackear
	void startTracking();                         // Comienza tracking
	void startCalibration();                         // Comienza calibracion
	int getTrackedIndex();                        // Indice del segmento trackeado (en la lista de segmentos)
	SegmentFeatures& getTrackedSegment() {return trackedSegment;}
	int getNearIndex() {return nearIndex;}

	void restartTrackingWithIndex(int);  // Reiniciar tracking con indice de segmento
	int restartTrackingWithAngle(float); // Reiniciar tracking con angulo de segmento

	TrackingStatus getTrackingStatus(){return trackingStatus;}
	//double getLastAngleBeforeOcclusionRisk() {return lastAngleBeforeOcclusionRisk;}
	//double getLastRadiusBeforeOcclusionRisk() {return lastRadiusBeforeOcclusionRisk;}
private:
	// Caracteristicas del laser
	int numData;      // Numero de datos leidos por el laser
	float resolution; // Resolucion del laser (grados)

	// Estado del tracker
	int state;        // Estado del PersonTracker (calibrando o trackeando)
	int cycles;       // Numero de veces que processNewReading ha sido llamado

	// Segmentacion
	std::vector<Segment> segments;
	void segment(long *data);
	void getFeatures(Segment& s,float* meanA,float* meanR,float* deltaA); // Obtiene caracteristicas de segmento

	// Objeto trackeado
	SegmentFeatures trackedSegment; // Segmento trackeado (o por trackear, si state == ST_CALIBRATING)
	int trackedIndex;               // Indice de segmento trackeado (en el std::vector segments)

	// Objeto mas cercano
	SegmentFeatures nearSegment;
	int nearIndex;

	// Estado: calibracion
	bool calibrationOk;
	void calibrate();

	// Estado: tracking
	void track();
	TrackingStatus trackingStatus;
	//float lastAngleBeforeOcclusionRisk;
	//float lastRadiusBeforeOcclusionRisk;
	void trackFeatures(SegmentFeatures& sf, int& trackIndex, bool updateFeatures, int radius);

	// Estado: recover
	int numCyclesToRecover;

	void doTransition(TrackingStatus);
	int cyclesInCurrentState;

	//void scheduleRestartWithAngle(float);
	//void doScheduledRestarts();
	//bool scheduled;
	//float scheduledAngle;

	// Utilidad general
	int indexForAngle(float);

	// Logging
	FILE* log;
};
