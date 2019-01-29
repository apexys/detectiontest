#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <string>
#include "src/markerlist.h"
#include <c_api.h>
#define M_PI 3.14159265358924

	static mrvision::MarkerList MARKERLIST;

	class DetectedMarker {

	public:
		cv::Mat mMarkerImage;
		float mMarkerDirection;
		cv::Point mMarkerPosition;
		std::vector<bool> mMarker;
		int mMarkerId;
		DetectedMarker(cv::Mat aMarkerImage, float aMarkerDirection, cv::Point aMarkerPosition);
	};

	DetectedMarker::DetectedMarker(cv::Mat aMarkerImage, float aMarkerDirection, cv::Point aMarkerPosition) :
		mMarkerImage(aMarkerImage),
		mMarkerDirection(aMarkerDirection),
		mMarkerPosition(aMarkerPosition),
		mMarker(),
		mMarkerId(-1)
	{}

	bool identifyMarker(DetectedMarker& aNotYetFoundMarker);

	int mIteration = 0;

	int mThreshold = 220;
	int mMinSize = 8, mMaxSize = 15;
	int mDetectionAreaMaxX = 640, mDetectionAreaMaxY = 480, mDetectionAreaMinX = 0, mDetectionAreaMinY = 0;
	bool isRunning = false, mStatus = false;

	cv::Mat mBinaryImage;
	std::vector<cv::Point> mPoints, mArea, mRealPoints;
	int mCounterI = 0, mCounterJ = 0, mCounterK = 0, mCounterL = 0;

	int mMaxRangeBetweenPoints = 360, mMinRangeBetweenPoints = 25; //quadratisch
	std::vector<std::vector<int> > mDistanceBetweenPoints;
	std::vector<std::vector<cv::Point> > mFoundCombinations;

	int mPointA = 0, mPointB = 0, mPointC = 0, mPointD = 0;
	int mDistanceAB = 0, mDistanceBC = 0, mDistanceCD = 0, mDistanceAD = 0, mDistanceAC = 0, mDistanceBD = 0;
	bool mDistanceABtest = false, mDistanceBCtest = false, mDistanceCDtest = false, mDistanceADtest = false, mDistanceACtest = false, mDistanceBDtest = false;
	// Read CameraFile - maybe move to Camera
	cv::Mat mCameraMatrix;
	cv::Mat mDistortionMatrix;
	cv::Size mCameraSize;


	int notFound = 0, notIdentified = 0, wrong = 0;

	int foundMarkerIds[64];
	int realFoundMarkerIds[64];

	void detectingMarkerInImage(const cv::Mat& aImage) {

		std::vector<DetectedMarker> vFoundMarkers;

		mBinaryImage = aImage > mThreshold;

		mRealPoints.clear();
		mPoints.clear();

		try {
			cv::findNonZero(
				mBinaryImage(
					cv::Rect(
						cv::Point(std::max(0, mDetectionAreaMinX), std::max(0, mDetectionAreaMinY)),
						cv::Point(std::min(mBinaryImage.size().width, mDetectionAreaMaxX), std::min(mBinaryImage.size().height, mDetectionAreaMaxY))
					)
				),
				mPoints);
		}
		catch (cv::Exception vNoPointsDetected) {
			std::cout << "No Points detected!\n" << std::endl;
			isRunning = false;
			return;
		}



		while (!mPoints.empty()) {
			mCounterI = 0;
			mArea.clear();

			mArea.push_back(mPoints.back());
			mPoints.pop_back();

			while (mCounterI < mArea.size()) {

				for (mCounterJ = mPoints.size() - 1; mCounterJ >= 0; --mCounterJ) {

					if (mArea[mCounterI].y == mPoints[mCounterJ].y && mArea[mCounterI].x + 1 == mPoints[mCounterJ].x ||
						mArea[mCounterI].y == mPoints[mCounterJ].y && mArea[mCounterI].x - 1 == mPoints[mCounterJ].x ||
						mArea[mCounterI].y + 1 == mPoints[mCounterJ].y && mArea[mCounterI].x == mPoints[mCounterJ].x ||
						mArea[mCounterI].y - 1 == mPoints[mCounterJ].y && mArea[mCounterI].x == mPoints[mCounterJ].x /*||
						mArea[mCounterI].y + 1 == mPoints[mCounterJ].y && mArea[mCounterI].x + 1 == mPoints[mCounterJ].x ||
						mArea[mCounterI].y - 1 == mPoints[mCounterJ].y && mArea[mCounterI].x - 1 == mPoints[mCounterJ].x ||
						mArea[mCounterI].y - 1 == mPoints[mCounterJ].y && mArea[mCounterI].x + 1 == mPoints[mCounterJ].x ||
						mArea[mCounterI].y + 1 == mPoints[mCounterJ].y && mArea[mCounterI].x - 1 == mPoints[mCounterJ].x */) {

						mArea.push_back(mPoints[mCounterJ]);
						mPoints.erase(mPoints.begin() + mCounterJ);

					}

				}
				++mCounterI;
			}
			//Calculate middle point of blob
			mRealPoints.push_back(cv::Point());
			for (cv::Point vPoint : mArea) {

				mRealPoints.back() += vPoint;

			}
			mRealPoints.back().x /= mArea.size();
			mRealPoints.back().y /= mArea.size();
		}

		//Bei zu vielen Points gib auf, Kamera falsch eingestellt oder zu viel Müll auf dem Tisch
		if (mRealPoints.size() > 400) {
			isRunning = false;
			return;

		}

		cv::Mat out3;
		cv::cvtColor(aImage, out3, cv::COLOR_GRAY2BGR);

		for (auto p : mRealPoints) {
			cv::circle(out3, p, 1, cv::Scalar(0, 0, 255), 2);
		}

		cv::imshow("Dots", out3);



		for (int vI = 0; vI < mRealPoints.size(); ++vI) {
			mRealPoints[vI].x += mDetectionAreaMinX;
			mRealPoints[vI].y += mDetectionAreaMinY;
		}

		mFoundCombinations.clear();
		if (mDistanceBetweenPoints.size() < mRealPoints.size()) {
			mDistanceBetweenPoints.resize(mRealPoints.size());
			for (mCounterI = 0; mCounterI < mRealPoints.size(); ++mCounterI) {
				mDistanceBetweenPoints[mCounterI].resize(mRealPoints.size());
			}
		}

		for (mCounterI = 0; mCounterI < mRealPoints.size(); ++mCounterI) {
			for (mCounterJ = 0; mCounterJ < mRealPoints.size(); ++mCounterJ) {
				mDistanceBetweenPoints[mCounterI][mCounterJ] = std::pow(mRealPoints[mCounterI].x - mRealPoints[mCounterJ].x, 2) + std::pow(mRealPoints[mCounterI].y - mRealPoints[mCounterJ].y, 2);
				mDistanceBetweenPoints[mCounterJ][mCounterI] = mDistanceBetweenPoints[mCounterI][mCounterJ];
			}

		}


		std::stringstream strstrm;

		auto almostEquals = [delta = 0.25](double a, double b) {return std::abs(a - b) < (std::min(a, b)); };
		auto inRange = [min = 11.0, max = 17.0](double val){return (val >= min * min && val <= max * max); };
		int ctr = 0;

		for (int a = 0; a < mRealPoints.size(); a++) {
			for (int b = 0; b < mRealPoints.size(); b++) {
				if (a == b) continue;
				double ab = mDistanceBetweenPoints[a][b];
				if (!inRange(ab)) continue;
				for (int c = 0; c < mRealPoints.size(); c++) {
					if (a == c || b == c) continue;
					double bc = mDistanceBetweenPoints[b][c];
					if (!inRange(bc)) continue;
					double ac = mDistanceBetweenPoints[a][c];
					ctr++;
					double abbc = ab + bc;

					strstrm << ctr << "|   " << "ab: " << ab << "  bc: " << bc << "  ac: " << ac << "  abbc: " << abbc << std::endl;

					if (almostEquals(ab, bc) && almostEquals(abbc, ac)) {

						//std::cout << ctr << "|   " << "ab: " << ab << "  bc: " << bc << "  ac: " << ac << "  abbc: " << abbc << " FOUND" << std::endl;
						cv::Point d;
						if (mRealPoints[c].x == mRealPoints[a].x) {
							d = cv::Point(mRealPoints[a].x + (mRealPoints[a].x - mRealPoints[b].x), mRealPoints[a].y);
						}
						else {
							double m = ((double)(mRealPoints[c].y - mRealPoints[a].y)) / (mRealPoints[c].x - mRealPoints[a].x);
							double m_p = -1 / m;
							double b_ac = (double)mRealPoints[a].y - (m * mRealPoints[a].x);
							double b_p = (double)mRealPoints[b].y - (m_p * mRealPoints[b].x);

							double x = (b_p - b_ac) / (m - m_p);

							double d_x = x + (x - mRealPoints[b].x);
							double d_y = d_x * m_p + b_p;
							d = cv::Point(d_x, d_y);
						}
						std::vector<cv::Point> vCombination;
						vCombination.push_back(mRealPoints[a]);
						vCombination.push_back(mRealPoints[b]);
						vCombination.push_back(mRealPoints[c]);
						vCombination.push_back(d);
						//std::cout << "Points: " << mRealPoints[a] << " - " << mRealPoints[b] << " - " << mRealPoints[c] << " - " << d << std::endl;
						mFoundCombinations.push_back(vCombination);
					}
				}
			}
		}

		if (mFoundCombinations.size() == 0) {
			//std::cout << strstrm.str();
		}

		cv::Mat vRegionOfInterest, vRegionOfInterestImage, vMarkerImage, vRotationMatrix;

		std::vector<cv::Point> vPotentialMarker;
		cv::RotatedRect vRotationBox;
		bool markerFound = false;
		for (std::vector<cv::Point> vMarker : mFoundCombinations) {

			try {

				vPotentialMarker.clear();
				vPotentialMarker.push_back(vMarker[0]);
				vPotentialMarker.push_back(vMarker[1]);
				vPotentialMarker.push_back(vMarker[2]);
				vPotentialMarker.push_back(vMarker[3]);

				vRotationBox = cv::minAreaRect(cv::Mat(vPotentialMarker));
				vRegionOfInterest = mBinaryImage(vRotationBox.boundingRect());
				vRotationMatrix = cv::getRotationMatrix2D(cv::Point(vRegionOfInterest.size())*0.5, vRotationBox.angle, 1);

				cv::warpAffine(vRegionOfInterest, vRegionOfInterestImage, vRotationMatrix, vRegionOfInterest.size());
				cv::resize(vRegionOfInterestImage, vMarkerImage, cv::Size(), 1, 1);

				if (vMarkerImage.data != NULL && !vMarkerImage.empty()) {
					//emit showPotentialMarker(vMarkerImage, mIteration);
				}

				DetectedMarker vUnDetectedMarker(vMarkerImage, vRotationBox.angle, vRotationBox.center);

				cv::Mat outputImage(aImage);

				cv::Point2f vertices[4];
				vRotationBox.points(vertices);
				for (int i = 0; i < 4; i++)
					line(outputImage, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 255, 255), 1);

				cv::imshow("Found", outputImage);
				cv::imshow("Marker", vMarkerImage);

				if (identifyMarker(vUnDetectedMarker)) {
					std::cout << "Marker identified: " << vUnDetectedMarker.mMarkerId << std::endl;//vFoundMarkers.push_back(vUnDetectedMarker);
					if (vUnDetectedMarker.mMarkerId != 7) {
						wrong++;
					}
					foundMarkerIds[vUnDetectedMarker.mMarkerId] ++;
					markerFound = true;
					vFoundMarkers.push_back(vUnDetectedMarker);
				}

			}
			catch (cv::Exception dinge) {
				//std::cout << dinge.what() << std::endl;
			}
		}

		auto markerInRange = [min = 0.0, max = 15.0](DetectedMarker m1, DetectedMarker m2){double val = std::pow(m1.mMarkerPosition.x - m2.mMarkerPosition.x, 2) + std::pow(m1.mMarkerPosition.y - m2.mMarkerPosition.y, 2);   return (val >= min * min && val <= max * max); };

		std::vector<std::vector<DetectedMarker>> markerGroups;
		for (auto marker : vFoundMarkers) {
			bool markerAssigned = false;
			for (auto group : markerGroups) {
				for (auto marker2 : group) {
					if (markerInRange(marker, marker2)) {
						group.push_back(marker);
						markerAssigned = true;
						break;
					}
				}
				if (markerAssigned) {
					break;
				}
			}
			if (!markerAssigned) {
				markerGroups.push_back(std::vector<DetectedMarker>{marker});
			}
		}

		std::vector<DetectedMarker> vRealFoundMarkers;

		for (auto group : markerGroups) {
			std::vector<int> foundMarkerIds(64, 0);
			for (auto marker : group) {
				foundMarkerIds[marker.mMarkerId]++;
			}

			int max = 0, maxId = 0;
			for (int i = 0; i < 64; i++) {
				if (foundMarkerIds[i] > max) {
					max = foundMarkerIds[i];
					maxId = i;
				}
			}

			auto result = std::find_if(group.begin(), group.end(), [&](DetectedMarker el) {return el.mMarkerId == maxId; });
			vRealFoundMarkers.push_back(*result);
			realFoundMarkerIds[maxId] ++;
		}


		if (!markerFound) {
			std::cout << "No marker found " << std::endl;
			notIdentified++;
			//cv::waitKey(0);
		}

		if (mFoundCombinations.size() == 0) {
			//cv::waitKey(0);
			std::cout << "No combinations" << std::endl;
			notFound++;
			//emit markersDetected(vFoundMarkers);
		}

		mIteration++;

		isRunning = false;

		std::cout << "Not found: " << notFound << ", not identified: " << notIdentified << ", wrong: " << wrong << std::endl;
	}


	int vtr2;

	bool identifyMarker(DetectedMarker& aNotYetFoundMarker) {

		const auto imsize = 100.0;
		const auto marker_threshold = 0.25;

		cv::Mat vResizedImage(imsize, imsize, aNotYetFoundMarker.mMarkerImage.type());
		cv::resize(aNotYetFoundMarker.mMarkerImage, vResizedImage, vResizedImage.size(), 0, 0, cv::INTER_LINEAR);

		cv::Mat smallImage(32, 32, aNotYetFoundMarker.mMarkerImage.type());
		cv::resize(aNotYetFoundMarker.mMarkerImage, smallImage, smallImage.size(), 0, 0, cv::INTER_LINEAR);

		std::stringstream file;
		file << "../beta/img_" << vtr2++ << ".png";
		cv::imwrite(file.str(), smallImage);

		cv::Mat vBW = vResizedImage > 170;

		cv::Mat canny;

		cv::Canny(vBW, canny, 0, 255);
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(canny, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
		std::vector<std::vector<cv::Point>> contours_poly(contours.size());
		std::vector<cv::Rect> rawBlobs(contours.size());

		for (size_t i = 0; i < contours.size(); i++) {
			rawBlobs[i] = boundingRect(contours[i]);
		}

		if (rawBlobs.size() < 1) return false;

		cv::Rect upper_left = rawBlobs[0],
			lower_left = rawBlobs[0],
			upper_right = rawBlobs[0],
			lower_right = rawBlobs[0];


		auto euclidean = [](double edge_x, double edge_y, cv::Rect point) {return std::pow((edge_x - point.x), 2.0) + std::pow((edge_y - point.y), 2); };

		for (auto r : rawBlobs) {
			if (euclidean(0, 0, r) <= euclidean(0, 0, upper_left)) {
				upper_left = r;
			}
			if (euclidean(imsize, 0, r) <= euclidean(imsize, 0, upper_right)) {
				upper_right = r;
			}
			if (euclidean(0, imsize, r) <= euclidean(0, imsize, lower_left)) {
				lower_left = r;
			}
			if (euclidean(imsize, imsize, r) <= euclidean(imsize, imsize, lower_right)) {
				lower_right = r;
			}

		}

		cv::Rect imbounds(upper_left.x + upper_left.width, upper_left.y + upper_left.height, upper_right.x - (upper_left.x + upper_left.width), lower_left.y - (upper_left.y + upper_left.height));

		cv::Mat vMarker = vBW(imbounds);

		int width = vMarker.size().width / 3,
			height = vMarker.size().height / 3;

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				cv::Mat vPart = vMarker(cv::Rect(j * width, i * height, width, height));
				aNotYetFoundMarker.mMarker.push_back(cv::countNonZero(vPart) > (width * height * marker_threshold));
			}
		}

		int vRotations = 0;
		for (mrvision::Marker vTest : MARKERLIST.getMarker()) {
			if (vTest.compareTo(aNotYetFoundMarker.mMarker, vRotations)) {
				aNotYetFoundMarker.mMarkerDirection += 270 - vRotations * 90;
				aNotYetFoundMarker.mMarkerId = vTest.getId();

				return true;
			}
		}

		return false;
	}


	int main(int argc, char** argv)
	{
		std::cout << "Hello from TensorFlow C library version " << TF_Version() << std::endl;
		/*
		int vCounter = 0;
		do {
			try {

				cv::Mat image;
				std::stringstream file;
				file << "../images_with_marker/10/image" << vCounter++ << ".png";
				std::cout << file.str() << std::endl;
				image = cv::imread(file.str(), cv::IMREAD_GRAYSCALE);   // Read the file

				cv::imshow("Raw image", image);

				detectingMarkerInImage(image);


				cv::waitKey(1);

			}
			catch (...) {

			}

		} while (vCounter <= 1847);

		std::cout << "Markers found: " << std::endl;

		for (int i = 0; i < 64; i++) {
			if (foundMarkerIds[i] > 0) {
				std::cout << i << ": fake: " << foundMarkerIds[i] << ", real: " << realFoundMarkerIds[i] << std::endl;
			}
		}

		std::cout << "Total: fake: " << std::accumulate(std::begin(foundMarkerIds), std::end(foundMarkerIds), 0) << ", real: " << std::accumulate(std::begin(realFoundMarkerIds), std::end(realFoundMarkerIds), 0) << std::endl;
		*/
		while (1);
		return 0;
	}