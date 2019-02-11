#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <string>
#include "src/markerlist.h"
#include <c_api.h>
#include "src/neural.h"
#include <stdio.h>
#include <direct.h>
#define M_PI 3.14159265358924

	static mrvision::MarkerList MARKERLIST;

	class DetectedMarker {

	public:
		cv::Mat mMarkerImage;
		float mMarkerDirection;
		cv::Point mMarkerPosition;
		std::vector<bool> mMarker;
		float mMarkerConfidence;
		int mMarkerId;
		DetectedMarker(cv::Mat aMarkerImage, float aMarkerDirection, cv::Point aMarkerPosition);
	};

	DetectedMarker::DetectedMarker(cv::Mat aMarkerImage, float aMarkerDirection, cv::Point aMarkerPosition) :
		mMarkerImage(aMarkerImage),
		mMarkerDirection(aMarkerDirection),
		mMarkerPosition(aMarkerPosition),
		mMarker(),
		mMarkerId(-1),
		mMarkerConfidence(0.0)
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

		cv::Mat outputImage;
		cv::cvtColor(aImage, outputImage, cv::COLOR_GRAY2BGR);


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

				cv::Point2f vertices[4];
				vRotationBox.points(vertices);
				for (int i = 0; i < 4; i++)
					cv::line(outputImage, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);

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


		cv::imshow("Found", outputImage);

		auto isNear = [max_distance = 10 * 10](DetectedMarker m1, DetectedMarker m2) {float distance_sq = std::pow(m1.mMarkerPosition.x - m2.mMarkerPosition.x, 2) + std::pow(m1.mMarkerPosition.y - m2.mMarkerPosition.y, 2); return distance_sq <= max_distance;};

		std::vector<DetectedMarker> ActuallyRealFoundMarkers;

		if (vFoundMarkers.size()) {
			while (vFoundMarkers.size()) {
				std::vector<DetectedMarker> cluster;
				cluster.push_back(vFoundMarkers.back());
				vFoundMarkers.pop_back();
				
				for (int i = 0; i < vFoundMarkers.size(); i++) {
					if (isNear(vFoundMarkers[i],cluster.front())) {
						cluster.push_back(vFoundMarkers[i]);
						vFoundMarkers.erase(vFoundMarkers.begin() + i, vFoundMarkers.begin() + i + 1);
						i--;
					}
				}

				DetectedMarker best_marker = cluster.front();
				for (auto m : cluster) {
					if (m.mMarkerConfidence > best_marker.mMarkerConfidence) {
						best_marker = m;
					}
				}
				ActuallyRealFoundMarkers.push_back(best_marker);
			}
		}

		cv::Mat out4;
		cv::cvtColor(aImage, out4, cv::COLOR_GRAY2BGR);

		cv::line(out4, cv::Point(30, 30), cv::Point(30 + std::cos(0 / 180.0 * 3.141) * 20, 30 + std::sin(0 / 180.0 * 3.141) * 20), cv::Scalar(0, 0, 255), 3);
		cv::line(out4, cv::Point(30, 30), cv::Point(30 + std::cos(90 / 180.0 * 3.141) * 20, 30 + std::sin(90 / 180.0 * 3.141) * 20), cv::Scalar(0, 255, 0), 3);


		for (auto m : ActuallyRealFoundMarkers) {
			std::cout << m.mMarkerId << " " << m.mMarkerDirection << " " << m.mMarkerConfidence << std::endl;
			cv::circle(out4, m.mMarkerPosition, 2, cv::Scalar(0, 0, 255),3);
			cv::line(out4, m.mMarkerPosition, cv::Point(m.mMarkerPosition.x + std::cos((m.mMarkerDirection) / 180.0 * 3.141) * 20, m.mMarkerPosition.y + std::sin((m.mMarkerDirection)/ 180.0 * 3.141) * 20), cv::Scalar(0, 0, 255),3);
		}

		cv::imshow("Markers", out4);

		mIteration++;

		isRunning = false;
	}


	model_t modelvar;
	bool model_loaded = false;
	model_t getModel() {
		if (!model_loaded) {
			//https://gist.github.com/asimshankar/7c9f8a9b04323e93bb217109da8c7ad2
			if (!ModelCreate(&modelvar, "./frozen_model.pb")) {
				std::cout << "Model loading failed" << std::endl;
			}
			else {
				model_loaded = true;
			}
		}
		return modelvar;
	}

	int vtr2;

	bool identifyMarker(DetectedMarker& aNotYetFoundMarker) {
		try{
			const auto imsize = 100.0;
			const auto marker_threshold = 0.25;

			cv::Mat vResizedImage(imsize, imsize, aNotYetFoundMarker.mMarkerImage.type());
			cv::resize(aNotYetFoundMarker.mMarkerImage, vResizedImage, vResizedImage.size(), 0, 0, cv::INTER_LINEAR);

			cv::Mat smallImage(32, 32, aNotYetFoundMarker.mMarkerImage.type());
			cv::resize(aNotYetFoundMarker.mMarkerImage, smallImage, smallImage.size(), 0, 0, cv::INTER_LINEAR);

			cv::Mat img2;
			smallImage.convertTo(img2, CV_32FC1);

			float matData[32 * 32];

			for (int y = 0; y < 32; y++) {
				for (int x = 0; x < 32; x++) {
					matData[y * 32 + x] = img2.at<float>(y, x);
				}
			}

			const int64_t dims[4] = { 1, 32, 32 , 1 };
			const size_t nbytes = 32 * 32 * sizeof(float);
			TF_Tensor* t = TF_AllocateTensor(TF_FLOAT, dims, 4, nbytes);
			memcpy(TF_TensorData(t), matData, nbytes);
			model_t model = getModel();
			TF_Output inputs[1] = { model.input };
			TF_Tensor* input_values[1] = { t };
			TF_Output outputs[1] = { model.output };
			TF_Tensor* output_values[1] = { NULL };

			TF_SessionRun(model.session, //Session
				NULL, //Run options
				inputs /*input tensor*/, input_values /*input values*/, 1 /*ninputs*/,
				outputs /*output tensor*/, output_values /*output values*/, 1 /*noutputs*/,
				/* No target operations to run */
				NULL /*target operations*/, 0 /*number of targets*/, NULL /*run metadata*/, model.status /*output status*/);
			TF_DeleteTensor(t);
			if (!Okay(model.status)) return false;
			float* predictions = (float*)malloc(sizeof(float) * 16);
			memcpy(predictions, TF_TensorData(output_values[0]), sizeof(float) * 16);
			TF_DeleteTensor(output_values[0]);

			char* ids[] = { "7", "8", "9", "10" };
			char* dirs[] = { "up  ", "down", "left", "right" };
			float rots[] = { 270.0, 90.0, 180.0, 0.0 };
			bool found = false;
			float rot = aNotYetFoundMarker.mMarkerDirection;
			std::cout << std::endl << std::endl << "Rot: " << rot << std::endl;
			for (int i = 0; i < 16; ++i) {
				printf("%s %s  %f  - ", ids[i / 4], dirs[i % 4], predictions[i]);
				if (predictions[i] > 0.95 && aNotYetFoundMarker.mMarkerConfidence < predictions[i]) {
					found = true;
					aNotYetFoundMarker.mMarkerDirection = (int)((rot + rots[i % 4]) + 360) % 360;
					aNotYetFoundMarker.mMarkerId = (i / 4) + 7;
					aNotYetFoundMarker.mMarkerConfidence = predictions[i];
				}
			}
			free(predictions);
			std::cout << std::endl;
			return found;
		}
		catch (const std::exception& e) {
			std::cout <<"ERRÖHR" << e.what() << std::endl;
		}

	
		return false;
	}


	int main(int argc, char** argv)
	{
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


				cv::waitKey(30);

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
		
		return 0;
	}