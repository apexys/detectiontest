#ifndef _mrvision_marker_h
#define _mrvision_marker_h

#include <vector>

namespace mrvision {

class Marker {

	// All 4 rotations of the marker 0°, 90°, 180°, 270° clockwise
	std::vector< std::vector<bool> > mRotationsOfMarker;
	int mId, mWidth, mHeight;
	int mNumberOfTrues;

public:
	Marker( std::vector<bool> aMarker, int aId, int aWidth = 3, int aHeight = 3 );
	virtual ~Marker();

	bool compareTo( std::vector<bool> aRealMarker, int &aRotations );

	int getId() const;
	int getWidth() const;
	int getHeight() const;
	std::vector<bool> getMakerRotation( int aRotation = 0 ) const;

	void print();

private:
	int countTrues( const std::vector<bool>& aMarker );

};


}

#endif // _mrvision_marker_h
