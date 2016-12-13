#include "tracking/laser_tracker/PersonTracker.h"
#include <algorithm>ersonTracker::PersonTracker(int numData, float resolution) : numData(numData),resolution(resolution)
{
	cycles = 0;
	state = ST_CALIBRATING;
	trackedIndex = -1;
	//scheduled = false;
}

PersonTracker::~PersonTracker(void)
{
}

void PersonTracker::processNewReading(long* data, int data_max) {
	segments.clear();
	this->segment(data);

	//if(segments.size() > 0) {
	//	printf("Length[0]=%f\n", segments[0].getExtent(), segments[0]);
	//}

	if(state == ST_CALIBRATING) {
		// Obtener objeto a trackear
		this->calibrate();

	} else if(state == ST_TRACKING) {
		// Trackear
		this->track();
	}

	cycles++;
}

const std::vector<Segment>& PersonTracker::getSegments() {
	return this->segments;
}

bool PersonTracker::isCalibrationOk() {
	return this->calibrationOk;
}

int PersonTracker::getTrackedIndex() {
	return this->trackedIndex;
}

void PersonTracker::startTracking() {
	state = ST_TRACKING;
	//calibrationOk = false;
	calibrationOk = true;
	doTransition(TRACK_OK); // Supposing that isCalibrationOk == true
}

void PersonTracker::startCalibration() {
	state = ST_CALIBRATING;
	calibrationOk = false;
}

void PersonTracker::segment(long *data) {
#if DEBUG_OUTPUT
	// Abro archivo para escribir lecturas (para efectos de debugging)
	char filename[256];
	sprintf(filename, "debugdata\\%05d_laser.txt", cycles);
	FILE* f = fopen(filename, "w+");

	fprintf(f, "---------------------------------------------------------\n");
	fprintf(f, "----------------------LASER SEGMENTS---------------------\n");
	fprintf(f, "---------------------------------------------------------\n");
#endif

	// Crear primer segmento
	Segment s;
	segments.push_back(s);
	int segindex = 0;        // Indice al ultimo segmento

	// Variables que contienen ultimo angulo y radio validos
	long lastR;
	float lastA;

	// Indice sobre los datos
	int i=0;
	// Angulo actual
	float angle = -N135;

	// Encontrar primer dato valido
	for( ; i<numData; i++, angle+=resolution) {
		if(data[i] == -1) {
			#if DEBUG_OUTPUT
			fprintf(f,"Data %f=>%d\n", angle, data[i]);
			#endif
			continue;
		}

		lastR = data[i];
		lastA = angle;
		break;
	}

	if(i == numData-1) {
		segments.clear();
		return;
	}

	#if DEBUG_OUTPUT
	fprintf(f,"---------SEGMENT START------------\n");
	fprintf(f,"Data %f=>%d\n", angle, data[i]);
	#endif

	segments[segindex].addPoint(angle, data[i]);
	i++; angle+=resolution;

	for( ; i<numData; i++, angle+=resolution) {
		// Saltarse datos invalidos
		if(data[i] == -1) {
			#if DEBUG_OUTPUT
			fprintf(f,"Data %f=>%d\n", angle, data[i]);
			#endif
			continue;
		}

		// Ver a que distancia esta punto actual de ultimo punto
		float x = (float)sqrt(data[i]*data[i] + lastR*lastR - 2*data[i]*lastR*cos(Deg2Rad(resolution)));

		bool mustAddSegment = x > SEGMENT_DISTANCE_THRESHOLD;
		if(mustAddSegment) {
			// Poner en nuevo segmento
			Segment snew;
			snew.addPoint(angle,data[i]);
			segments.push_back(snew);
			segindex++;
		} else {
			// Poner en segmento actual
			segments[segindex].addPoint(angle, data[i]);
		}

		// Ver si descartar ultimo segmento
		if(mustAddSegment || i == numData-1) {
			int segmentToCheck = 0;
			// Si tuvo que agregar un segmento solo para el ultimo dato, descartamos este segmento
			if(mustAddSegment && i == numData-1) {
				#if DEBUG_OUTPUT
				fprintf(f,"---------SEGMENT DISCARDED (one reading only)------------\n");
				#endif
				segments.pop_back();
				--segindex;
				segmentToCheck = segindex;
			} else if(mustAddSegment) {
				segmentToCheck = segindex-1;
			} else if(i == numData-1) {
				segmentToCheck = segindex;
			}

			float segmentLength = (float)segments[segmentToCheck].getExtent();
			long maxDistance;
			if(segmentLength <= MIN_SEGMENT_LENGTH || segmentLength >= MAX_SEGMENT_LENGTH) {
				#if DEBUG_OUTPUT
				fprintf(f,"---------SEGMENT DISCARDED (too short/long=%f)------------\n", segmentLength);
				#endif
				// Descartar segmento demasiado largo
				segments.erase(segments.begin() + segmentToCheck);
				segindex--;
			} else if((maxDistance = segments[segmentToCheck].getMaxDistance()) >= MAX_SEGMENT_DISTANCE) {
				#if DEBUG_OUTPUT
				fprintf(f,"---------SEGMENT DISCARDED (too far=%d)------------\n", maxDistance);
				#endif
				// Descartar segmento con puntos demasiado alejados
				segments.erase(segments.begin() + segmentToCheck);
				segindex--;
			} else if(segments[segmentToCheck].distances.size() < MIN_SEGMENT_DATA_SIZE) {
				#if DEBUG_OUTPUT
				fprintf(f,"---------SEGMENT DISCARDED (too few data=%d)------------\n", segments[segmentToCheck].distances.size());
				#endif
				// Descartar segmento con muy pocos datos
				segments.erase(segments.begin() + segmentToCheck);
				segindex--;
			} else if(segments[segmentToCheck].getMaxAngle() <= MIN_ANGLE_RANGE || segments[segmentToCheck].getMinAngle() >= MAX_ANGLE_RANGE) {
				#if DEBUG_OUTPUT
				fprintf(f,"---------SEGMENT DISCARDED (out of interest zone, angles[%f,%f])------------\n",
					segments[segmentToCheck].getMinAngle(), segments[segmentToCheck].getMaxAngle());
				#endif

				// Descartar segmento fuera de angulos de interes
				segments.erase(segments.begin() + segmentToCheck);
				segindex--;
			} else {
				// Preservar segmento
				#if DEBUG_OUTPUT
				fprintf(f,"---------SEGMENT END (length=%f, maxDistance=%d)------------\n", segmentLength, maxDistance);
				#endif
			}
		}

		#if DEBUG_OUTPUT
		fprintf(f,"Data %f=>%d distToLast=%f\n", angle, data[i], x);
		#endif

		lastR = data[i];
		lastA = angle;
	}

#if DEBUG_OUTPUT
	fclose(f);
#endif
}

void PersonTracker::calibrate() {
#if DEBUG_OUTPUT
	// Abro archivo para escribir lecturas (para efectos de debugging)
	char filename[100];
	sprintf(filename, "debugdata\\%05d_tracking.txt", cycles);
	FILE* f = fopen(filename, "w+");
#endif

	int n = segments.size();
	for(int i=0;i<n;i++) {
		if(segments[i].hasAngle(0)) {
			//float minAngle = segments[i].getMinAngle();
			//float maxAngle = segments[i].getMaxAngle();
			//float minRadius = segments[i].getMinRadius();
			//float maxRadius = segments[i].getMinRadius();
			//
			//float ma = (minAngle+maxAngle)/2;
			//float mr = (minRadius+maxRadius)/2;
			//float da = maxAngle-minAngle;

			float ma, mr, da;
			getFeatures(segments[i], &ma, &mr, &da);

			trackedSegment.updateTo(ma, mr, da);
			calibrationOk = true;

			#if DEBUG_OUTPUT
			fprintf(f, "Calibration with: ma=%f, mr=%f, da=%f\n", ma, mr, da);
			fclose(f);
			#endif

			return;
		}
	}
	calibrationOk = false;

	#if DEBUG_OUTPUT
	fprintf(f, "No object to calibrate with\n");
	fclose(f);
	#endif
}

void PersonTracker::trackFeatures(SegmentFeatures& sf, int& trackIndex, bool updateFeatures, int radius) {
	#if DEBUG_OUTPUT
	fprintf(log,"[trackFeatures] TrackedSegment (ma,mr,da)=(%f,%f,%f)\n", sf.getMeanAngle(), sf.getMeanRadius(), sf.getDeltaAngle());
	#endif

	int n = segments.size();
	float minDist = std::numeric_limits<float>::infinity();

	float newMeanA, newMeanR, newDeltaA;
	float oldMeanA  = sf.getMeanAngle(),
		  oldMeanR  = sf.getMeanRadius(),
		  oldDeltaA = sf.getDeltaAngle();
	float wOld = 0, wNew = 1-wOld;

	for(int i=0;i<n;i++) {
		// Get features for tracking
		float meanA, meanR, deltaA;
		getFeatures(segments[i], &meanA, &meanR, &deltaA);

		// Get distance from candidate segment to tracked segment
		float displacement = (float)PolarDistance(oldMeanR, oldMeanA, meanR, meanA);

		if(displacement > radius) {
			#if DEBUG_OUTPUT
			fprintf(log,"[trackFeatures] candidate (ma,mr,da)=(%f,%f,%f) too physically far from segment (%f)\n", meanA, meanR, deltaA,displacement);
			#endif
		} else {
			float dist = sf.distanceTo(meanA, meanR, deltaA);

			#if DEBUG_OUTPUT
			fprintf(log,"[trackFeatures] candidate (ma,mr,da)=(%f,%f,%f) phys-distance=%f tuple-distance=%f\n", meanA, meanR, deltaA, displacement, dist);
			#endif

			if(dist < minDist) {
				minDist = dist;
				newMeanA = meanA;
				newMeanR = meanR;
				newDeltaA = deltaA;

				trackIndex = i;
			}
		}
	}

	// If we found a good candidate, update tracking
	if(minDist < MAX_TRACKING_FEATURES_DISTANCE) {
		#if DEBUG_OUTPUT
		fprintf(log,"[trackFeatures] => Segment (%f,%f,%f) is the closest (tuple-dist=%f)\n", newMeanA, newMeanR, newDeltaA, minDist);
		#endif

		if(updateFeatures) {
			sf.updateTo(
					wNew*newMeanA+wOld*oldMeanA,
					wNew*newMeanR+wOld*oldMeanR,
					wNew*newDeltaA+wOld*oldDeltaA);
		}
	} else {
		trackIndex = -1;

		#if DEBUG_OUTPUT
		fprintf(log,"[trackFeatures] => No matching object found (best tuple-dist=%f, threshold=%d)\n", minDist, MAX_TRACKING_FEATURES_DISTANCE);
		#endif
	}


}

void PersonTracker::doTransition(TrackingStatus s) {
	trackingStatus = s;
	cyclesInCurrentState = 0;
}

void PersonTracker::track() {
	#if DEBUG_OUTPUT
		// Abro archivo para escribir lecturas (para efectos de debugging)
		char filename[128];
		sprintf(filename, "debugdata\\%05d_tracking.txt", cycles);
		log = fopen(filename, "w+");
	#endif

	switch(trackingStatus) {
	case TRACK_OK: {
		#if DEBUG_OUTPUT
		fprintf(log,"[PersonTracker] Tracking OK: (r,theta)=(%f,%f)\n",
			trackedSegment.getMeanRadius(), trackedSegment.getMeanAngle());

		fprintf(log, "[PersonTracker] track object\n");
		#endif

		trackFeatures(trackedSegment, trackedIndex, true, MAX_DISPLACEMENT_BETWEEN_READINGS);

		if(trackedIndex >= 0) {
			// Find element that could occlude tracked segment
			float trA = trackedSegment.getMeanAngle(),
				  trR = trackedSegment.getMeanRadius();

			int n = segments.size();

			int   leftIndex, rightIndex;
			float leftA    , rightA    ;
			float leftR    , rightR    ;
			float leftDa   , rightDa   ;
			bool  leftOccl , rightOccl ;

			// Left element
			leftIndex = trackedIndex - 1;
			if(leftIndex >= 0) {
				getFeatures(segments[leftIndex], &leftA, &leftR, NULL);
			} else {
				leftA = MIN_ANGLE_RANGE;
				leftR = 4000;
			}

			leftDa   = trA - leftA;
			leftOccl = leftR*sin(Deg2Rad(leftDa)) <= OCCLUSION_RISK_DISTANCE && leftR <= trR;

			// Right element
			rightIndex = trackedIndex + 1;
			if(rightIndex <= n-1) {
				getFeatures(segments[rightIndex], &rightA, &rightR, NULL);
			} else {
				rightA = MAX_ANGLE_RANGE;
				rightR = 4000;
			}

			rightDa   = rightA - trA;
			rightOccl = rightR*sin(Deg2Rad(rightDa)) <= OCCLUSION_RISK_DISTANCE && rightR <= trR;

			//printf("left : (r,t)=(%f,%f) da=%f, leftOccl =%d\n", leftR, leftA, leftDa, leftOccl?1:0);
			//printf("right: (r,t)=(%f,%f) da=%f, rightOccl=%d\n", rightR, rightA, rightDa, rightOccl?1:0);

			// Decide which occlusion is more serious, if any
			bool hasOccl = leftOccl || rightOccl;

			if(hasOccl) {
				int occlIndex;
				if(leftOccl && rightOccl) {
					if(leftDa <= rightDa) {
						occlIndex = leftIndex;
					} else {
						occlIndex = rightIndex;
					}
				} else if(leftOccl) {
					occlIndex = leftIndex;
				} else {
					occlIndex = rightIndex;
				}

				float meanA, meanR, deltaA;
				getFeatures(segments[occlIndex], &meanA, &meanR, &deltaA);
				nearSegment.updateTo(meanA, meanR, deltaA);
				nearIndex = occlIndex;

				#if DEBUG_OUTPUT
				fprintf(log,"[PersonTracker] Transition to OCCLUSION_RISK: near object detected\n");
				#endif

				doTransition(TRACK_OCCLUSION_RISK);
			} else {
				// No occlusions, everything is ok
			}
		} else {
			#if DEBUG_OUTPUT
			fprintf(log,"[PersonTracker] Transition to LOST: no object detected\n");
			#endif

			doTransition(TRACK_LOST);
		}

		break;
	}

	case TRACK_OCCLUSION_RISK: {
		#if DEBUG_OUTPUT
		fprintf(log,"[PersonTracker] Tracking OCCLUSION_RISK: (r,theta)=(%f,%f)\n",
			trackedSegment.getMeanRadius(), trackedSegment.getMeanAngle());

		fprintf(log, "[PersonTracker] track object\n");
		#endif

		trackFeatures(trackedSegment, trackedIndex, false, MAX_DISPLACEMENT_BETWEEN_READINGS);
		#if DEBUG_OUTPUT
		fprintf(log, "[PersonTracker] track occluder\n");
		#endif
		trackFeatures(nearSegment   , nearIndex   , true , MAX_DISPLACEMENT_BETWEEN_READINGS);

		if(trackedIndex == nearIndex) {
			if(trackedIndex != -1) {
				if(cyclesInCurrentState >= MIN_CYCLES_OF_RISK_BEFORE_OCCLUSION) {
					#if DEBUG_OUTPUT
					fprintf(log,"[PersonTracker] Transition to OCCLUSION: object and occluder indistinguishable\n");
					#endif
					doTransition(TRACK_OCCLUSION);
				} else {
					// Some serious occlusions use this branch!!

					numCyclesToRecover = 10;
					#if DEBUG_OUTPUT
					fprintf(log,"[PersonTracker] Transition to RECOVER: not a serious risk of occlusion, wait %d frames\n", numCyclesToRecover);
					#endif
					doTransition(TRACK_RECOVER);
				}
			} else {
				#if DEBUG_OUTPUT
				//fprintf(log,"[PersonTracker] Transition to LOST: all objects lost\n");
				#endif
				//doTransition(TRACK_LOST);
			}
		} else {
			if(trackedIndex == -1) {
				#if DEBUG_OUTPUT
				fprintf(log,"[PersonTracker] Transition to OCCLUSION: only occluder visible\n");
				#endif
				doTransition(TRACK_OCCLUSION);
			} else if(nearIndex == -1) {
				#if DEBUG_OUTPUT
				fprintf(log,"[PersonTracker] Transition to OK: only main object visible\n");
				#endif
				doTransition(TRACK_OK);
			} else {
				// Si objetos estan muy lejos => pasar a ok
				float trR, trA, nrR, nrA;
				getFeatures(segments[trackedIndex], &trA, &trR, NULL);
				getFeatures(segments[nearIndex]   , &nrA, &nrR, NULL);
				double dist = PolarDistance(trR, trA, nrR, nrA);
				//fprintf(log,"[PersonTracker] Distance(%f,%f,%f,%f) %f\n", trR, trA, nrR, nrA, dist);
				if(dist > 1.3*OCCLUSION_RISK_DISTANCE) {
					#if DEBUG_OUTPUT
					fprintf(log,"[PersonTracker] Transition to OK: object and occluder far from each other (%f)\n", dist);
					#endif
					doTransition(TRACK_OK);
				} else {

				}
			}
		}

		break;
	}

	case TRACK_OCCLUSION: {
		#if DEBUG_OUTPUT
		fprintf(log,"[PersonTracker] Tracking OCCLUSION: (r,theta)=(%f,%f)\n",
			trackedSegment.getMeanRadius(), trackedSegment.getMeanAngle());

		fprintf(log, "[PersonTracker] track object\n");
		#endif

		trackFeatures(trackedSegment, trackedIndex, false, MAX_DISPLACEMENT_BETWEEN_READINGS);

		#if DEBUG_OUTPUT
		fprintf(log, "[PersonTracker] track occluder\n");
		#endif

		trackFeatures(nearSegment   , nearIndex   , true , MAX_DISPLACEMENT_BETWEEN_READINGS);

		if(trackedIndex == nearIndex) {
			if(trackedIndex != -1) {
				// If too many cycles in occlusion, just go to ok
				if(cyclesInCurrentState >= MAX_CYCLES_IN_OCCLUSION) {
					#if DEBUG_OUTPUT
					fprintf(log,"[PersonTracker] Transition to OK: waited too much\n");
					#endif
					doTransition(TRACK_OK);
				}
			}
		} else {
			numCyclesToRecover = 20;
			#if DEBUG_OUTPUT
			fprintf(log,"[PersonTracker] Transition to RECOVER: main object visible, wait %d frames\n", numCyclesToRecover);
			#endif
			doTransition(TRACK_RECOVER);
		}
		break;
	}

	case TRACK_LOST: {
		#if DEBUG_OUTPUT
		fprintf(log,"[PersonTracker] Tracking LOST\n");

		fprintf(log, "[PersonTracker] track object\n");
		#endif
		trackFeatures(trackedSegment, trackedIndex, false, MAX_DISPLACEMENT_BETWEEN_READINGS);

		if(trackedIndex != -1) {
			numCyclesToRecover = std::min(cyclesInCurrentState, 15);
			#if DEBUG_OUTPUT
			fprintf(log,"[PersonTracker] Transition to RECOVER: main object visible, wait %d frames\n", numCyclesToRecover);
			#endif
			doTransition(TRACK_RECOVER);
		}

		// Idea: si lleva mucho tiempo en este estado, reiniciar con segmento al centro

		break;
	}

	case TRACK_RECOVER: {
		//trackFeatures(trackedSegment, trackedIndex, false);
		#if DEBUG_OUTPUT
		fprintf(log,"[PersonTracker] Tracking RECOVER: frame %d of %d\n", cyclesInCurrentState, numCyclesToRecover);
		#endif

		if(cyclesInCurrentState >= numCyclesToRecover) {
			int excess = cyclesInCurrentState - numCyclesToRecover;
			int radius = std::min(MAX_DISPLACEMENT_BETWEEN_READINGS + excess*RECOVERY_RADIUS_INCREMENT,MAX_RECOVERY_RADIUS);

			#if DEBUG_OUTPUT
			fprintf(log, "[PersonTracker] Looking for candidates in a radius of %d mm\n", radius);
			#endif

			trackFeatures(trackedSegment, trackedIndex, false, radius);

			if(trackedIndex >= 0) {
				float tr, ta;
				getFeatures(segments[trackedIndex], &ta, &tr, NULL);
				float dist = (float)PolarDistance(tr, ta, trackedSegment.getMeanRadius(), trackedSegment.getMeanAngle());

				#if DEBUG_OUTPUT
				fprintf(log, "[PersonTracker] Transition to OK: restarted tracking\n");
				#endif
				restartTrackingWithIndex(trackedIndex);
				doTransition(TRACK_OK);
			}

			//float angle = trackedSegment.getMeanAngle();
			//if(restartTrackingWithAngle(angle)) {
			//	fprintf("[PersonTracker] Transition to OK: restarted tracking\n", angle);
			//	doTransition(TRACK_OK);
			//}
		}
		break;
	}

	}

	cyclesInCurrentState++;

	#if DEBUG_OUTPUT
		fclose(log);
	#endif
}

void PersonTracker::restartTrackingWithIndex(int index) {
	float ma,mr,da;
	getFeatures(segments[index],&ma,&mr,&da);

	trackedSegment.updateTo(ma,mr,da);
}

int PersonTracker::restartTrackingWithAngle(float angle) {
	int index = indexForAngle(angle);

	if(index != -1) {
		printf("[PersonTracker::restartTrackingWithAngle] Angle %f found in segment %d [%f,%f]\n", angle, index, segments[index].getMinAngle(), segments[index].getMaxAngle());
		restartTrackingWithIndex(index);
		return 1;
	} else {
		return 0;
	}
}

int PersonTracker::indexForAngle(float angle) {
	int n = segments.size();

	for(int i=0;i<n;++i) {
		if(segments[i].hasAngle(angle)) {
			return i;
		}
	}

	return -1;
}

void PersonTracker::getFeatures(Segment& s, float* meanA, float* meanR, float* deltaA) {
	float minAngle = (float)s.getMinAngle();
	float maxAngle = (float)s.getMaxAngle();
	float minRadius = (float)s.getMinRadius();
	float maxRadius = (float)s.getMinRadius();

	if(meanA != NULL) {
		*meanA  = (minAngle+maxAngle)/2;
	}

	if(meanR != NULL) {
		*meanR  = (minRadius+maxRadius)/2;
	}

	if(deltaA != NULL) {
		*deltaA = maxAngle-minAngle;
	}
}

//void PersonTracker::scheduleRestartWithAngle(float angle) {
//	scheduled = true;
//	scheduledAngle = angle;
//}

//void PersonTracker::doScheduledRestarts() {
//	if(scheduled) {
//		if(indexForAngle(scheduledAngle) != -1) {
//			scheduled = false;
//			printf("[PersonTracker] Restart tracking with angle %f\n", scheduledAngle);
//			restartTrackingWithAngle(scheduledAngle);
//		}
//	}
//}
