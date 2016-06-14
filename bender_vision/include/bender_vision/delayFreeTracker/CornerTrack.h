#ifndef CORNER_TRACK_H_
#define CORNER_TRACK_H_

class CornerTrack
{
public:
	double row;
	double col;
	double orig_row;
	double orig_col;
	bool valid;
	int frame;
	int orig_frame;
	int last_valid_frame;
	double t;
	double orig_t;
	int forget_counter;
};

#endif // CORNER_TRACK_H_
