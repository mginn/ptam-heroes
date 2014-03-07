#include <stdio.h>
#include <string.h>
#include <string>
#include <fstream>
#include <CL/opencl.h>

typedef struct clTrackerData
{
	cl_float v2Image[2];
	cl_float v2Found[2];    //this was a vector type in tracker data
	cl_float dSqrtInvNoise;
	cl_int bInImage;
	cl_int bPotentiallyVisible;
	cl_int bSearched;
	cl_int bFound;
	cl_int bDidSubPix;

}clTrackerData;

typedef struct clFinder
{
	float mv2CoarsePos[2];
	int mbTemplateBad;
	int mnSearchLevel;
	int LevelScale;
	int mnMaxSSD;
	int mnPatchSize;
	int mnTemplateSum;    // Cached pixel-sum of the coarse template
	int mnTemplateSumSq;
	int mbFound;
	int mirCenter[2];
	int mirPredictedPos[2];

	int manMeasAttempted[4];
	int manMeasFound[4];

}clFinder;

typedef struct clImage
{


	int image_size_x;
	int image_size_y;
	int image_stride;
	cl_uchar im[307200];                // The pyramid level pixels

}clImage;


typedef struct clTemplate
{


	int image_size_x;
	int image_size_y;
	int image_stride;
	cl_uchar im[81];                // The pyramid level pixels

}clTemplate;


typedef struct clLevel
{


	//float** vCorners;
		//float** vMaxCorners;  // The maximal FAST corners
		//int image_y;
		float vCorners[4000][2];
		float vMaxCorners[4000][2];
		int vCornerRowLUT[480];          // Row-index into the FAST corners, speeds up access

	            // The pyramid level pixels

}clLevel;


typedef struct clTrail
{

	int irCurrentPos[2];
	int irInitialPos[2];
	int  bFound;
}clTrail;


typedef struct clvCorners
{
	int vCon[2];

}clvCorners;

typedef struct clCam
{
	bool bFixed;
	  SE3<> se3CfW;
	  SE3<> se3CfWNew;
	  double m6U[6][6];          // Accumulator
	  double EpsilonA[6];   // Accumulator
	  //int nStartRow;
}clCam;

typedef struct clPoint
{
  //double v3Pos[3];
  //double v3PosNew[3];
  double m3V[3][3];          // Accumulator
  //double v3EpsilonB[3];   // Accumulator
  //double m3VStarInv[3][3];

  //int nMeasurements;
  //int nOutliers;
  //std::set<int> sCameras; // Which cameras observe this point?
  //std::vector<OffDiagScriptEntry> vOffDiagonalScript; // A record of all camera-camera pairs observing this point
}clPoint;

typedef struct clMeas
{
	int p;
	int c;
	bool bBad;
	double dErrorSquared;
	double v2Epsilon[2];
	double m2CamDerivs[2][2];
	double v3Cam[3];
	double m26A[2][6];
	double dSqrtInvNoise;
	double m23B[2][3];
}clMeas;

