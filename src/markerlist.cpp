
#include "src/markerlist.h"
#include "src/marker.h"

namespace mrvision {

MarkerList::MarkerList() :
		mMarker()
{
    for( int i = 1; i < 32; ++i ){
		if (i == 14) continue;
        int eins = 0, zwei = 0, vier = 0, acht = 0;
        eins = ((i % 2) + ((i >> 1) % 2) + ((i >> 3) % 2) + ((i >> 4) % 2)) % 2;
        zwei = ((i % 2) + ((i >> 2) % 2) + ((i >> 3) % 2)) % 2;
        vier = (((i >> 1) % 2) + ((i >> 2) % 2) + ((i >> 3) % 2)) % 2;
        acht = (((i >> 4) % 2)) % 2;
        mMarker.push_back( Marker( std::vector<bool>{ eins != 1, zwei != 1, (i % 2) != 1, vier != 1, ((i >> 1) % 2) != 1, ((i >> 2) % 2) != 1, ((i >> 3) % 2) != 1, acht != 1, ((i >> 4) % 2) != 1 }, i ) );
    }

/*
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,false,false,false,false,true}, 1 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,false,false,false,false,true}, 2 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,false,true,false,false,true}, 3 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,false,true,false,false,true}, 4 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,false,true,false,false,true}, 5 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,false,true,false,false,true}, 6 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,false,false,false,false,true}, 7 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,false,false,false,false,true}, 8 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,true,false,false,false,true}, 9 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,true,false,false,false,true}, 10 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,true,false,false,false,true}, 11 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,true,false,false,false,true}, 12 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,true,true,false,false,true}, 13 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,true,true,false,false,true}, 14 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,true,true,false,false,true}, 15 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,true,true,false,false,true}, 16 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,false,false,false,true,true}, 17 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,false,false,false,true,true}, 18 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,false,false,false,true,true}, 19 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,false,false,false,true,true}, 20 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,true,false,false,true,true}, 21 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,true,false,false,true,true}, 22 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,true,false,false,true,true}, 23 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,true,false,false,true,true}, 24 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,false,true,false,true,true}, 25 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,false,true,false,true,true}, 26 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,false,true,false,true,true}, 27 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,false,true,false,true,true}, 28 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,false,true,true,false,true,true}, 29 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,false,true,true,false,true,true}, 30 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,false,true,true,true,true,false,true,true}, 31 ) );*/
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true,true,true,true,false,true}, 32 ) );
	mMarker.push_back( Marker( std::vector<bool>{true,true,false,true,true,false,false,false,false}, 33 ) );
	/*
	mMarker.push_back( Marker( std::vector<bool>{true,true,true,true}, 1, 2, 2 ) );
	mMarker.push_back( Marker( std::vector<bool>{false,true,true,true}, 2, 2, 2 ) );
	mMarker.push_back( Marker( std::vector<bool>{false,false,true,true}, 3, 2, 2 ) );
	mMarker.push_back( Marker( std::vector<bool>{false,false,false,true}, 4, 2, 2 ) );
	mMarker.push_back( Marker( std::vector<bool>{false,false,false,false}, 5, 2, 2 ) );
	mMarker.push_back( Marker( std::vector<bool>{false,true,true,false}, 6, 2, 2 ) );*/
};

MarkerList::~MarkerList(){

};

void MarkerList::print(){

	for( Marker vMarker : mMarker ){

		vMarker.print();

	}

};


std::vector<Marker> MarkerList::getMarker(){

	return mMarker;

};

}

