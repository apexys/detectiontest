#include "src/marker.h"

#include <iostream>

namespace mrvision {

Marker::Marker( std::vector<bool> aMarker, int aId, int aWidth, int aHeight )
:	mId(aId), mWidth(aWidth), mHeight(aHeight)
{
	mNumberOfTrues = countTrues( aMarker );

	mRotationsOfMarker.push_back( aMarker );

	for( int i = 0; i < 3; i++){

		std::vector<bool> vHelper;

		vHelper.push_back( aMarker[6] );
		vHelper.push_back( aMarker[3] );
		vHelper.push_back( aMarker[0] );
		vHelper.push_back( aMarker[7] );
		vHelper.push_back( aMarker[4] );
		vHelper.push_back( aMarker[1] );
		vHelper.push_back( aMarker[8] );
		vHelper.push_back( aMarker[5] );
		vHelper.push_back( aMarker[2] );

		aMarker = vHelper;
		mRotationsOfMarker.push_back( aMarker );

	}

};

Marker::~Marker(){

};

bool Marker::compareTo( std::vector<bool> aRealMarker, int &aRotations ){

	if( mNumberOfTrues != countTrues( aRealMarker ) ){
		return false;
	}

	int vRotate = 0;

	for( std::vector<bool> vMarkerRotation : mRotationsOfMarker ){

		if( aRealMarker == vMarkerRotation ){
			aRotations = (4 - vRotate) % 4;
			return true;
		}

		vRotate++;

	}

	return false;

};

int Marker::getId() const{

	return mId;

};

int Marker::getWidth() const{

	return mWidth;

};

int Marker::getHeight() const{

	return mHeight;

};

std::vector<bool> Marker::getMakerRotation( int aRotation ) const{

    if( aRotation < 0 || aRotation > mRotationsOfMarker.size() ){
        return mRotationsOfMarker.at(0);
    }
    return mRotationsOfMarker.at(aRotation);

}


void Marker::print(){

	std::cout << "[" << mId << "] " << mNumberOfTrues << " truthiness" << std::endl;
	std::cout << mRotationsOfMarker[0][0] << mRotationsOfMarker[0][1] << mRotationsOfMarker[0][2] << " ";
	std::cout << mRotationsOfMarker[1][0] << mRotationsOfMarker[1][1] << mRotationsOfMarker[1][2] << " ";
	std::cout << mRotationsOfMarker[2][0] << mRotationsOfMarker[2][1] << mRotationsOfMarker[2][2] << " ";
	std::cout << mRotationsOfMarker[3][0] << mRotationsOfMarker[3][1] << mRotationsOfMarker[3][2];
	std::cout << std::endl;
	std::cout << mRotationsOfMarker[0][3] << mRotationsOfMarker[0][4] << mRotationsOfMarker[0][5] << " ";
	std::cout << mRotationsOfMarker[1][3] << mRotationsOfMarker[1][4] << mRotationsOfMarker[1][5] << " ";
	std::cout << mRotationsOfMarker[2][3] << mRotationsOfMarker[2][4] << mRotationsOfMarker[2][5] << " ";
	std::cout << mRotationsOfMarker[3][3] << mRotationsOfMarker[3][4] << mRotationsOfMarker[3][5];
	std::cout << std::endl;
	std::cout << mRotationsOfMarker[0][6] << mRotationsOfMarker[0][7] << mRotationsOfMarker[0][8] << " ";
	std::cout << mRotationsOfMarker[1][6] << mRotationsOfMarker[1][7] << mRotationsOfMarker[1][8] << " ";
	std::cout << mRotationsOfMarker[2][6] << mRotationsOfMarker[2][7] << mRotationsOfMarker[2][8] << " ";
	std::cout << mRotationsOfMarker[3][6] << mRotationsOfMarker[3][7] << mRotationsOfMarker[3][8];
	std::cout << std::endl;



}

int Marker::countTrues(const std::vector<bool>& aMarker) {

	int vTrues = 0;

	for (bool mBoolean : aMarker) {
		if (mBoolean) {
			vTrues++;
		}
	}

	return vTrues;

}



}
