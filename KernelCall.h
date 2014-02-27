/*
 * KernelCall.h
 *
 *  Created on: Jan 9, 2014
 *      Author: mainul
 */

#include <CL/opencl.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <fstream>
extern int globalCounterForKernelCall;

#include "CLCustomData.h"
using namespace std;

extern cl_context cxGPUContext;        // OpenCL context
extern cl_command_queue cqCommandQueue;// OpenCL command que
extern 	cl_platform_id cpPlatform;      // OpenCL platform
extern 	cl_device_id cdDevice;          // OpenCL device
extern 	cl_program cpProgram;           // OpenCL program
extern 	cl_kernel ckKernel;             // OpenCL kernel
extern 	cl_mem cmDevSrcA;               // OpenCL device source buffer A
extern 	cl_mem cmDevSrcB;               // OpenCL device source buffer B
extern 	cl_mem cmDevDst;                // OpenCL device destination buffer
extern size_t szGlobalWorkSize;        // 1D var for Total # of work items
extern size_t szLocalWorkSize;		// 1D var for # of work items in the work group
extern size_t szParmDataBytes;		// Byte size of context information
extern size_t szKernelLength;		// Byte size of kernel code
extern 	cl_int ciErr1, ciErr2;		// Error code var
extern 	char* cPathAndName ;      // var for full paths to data, src, etc.
extern 	char* cSourceCL ;         // Buffer to hold source for compilation




int SetContext();
int convertToString(char *filename, std::string& s);
int SearchForPointsKernel(char* kernelName,char * kernelFilename,
		int nRange,
		clTrackerData* TD,
		int* nFound,
		int vTDVectorSize,
		clFinder* Finder,
		int* manMeasAttempted,
		int LEVELSs,
		clImage* Imageg,
		clLevel* Levelg,
		clImage* mimTemplate,
		cl_int* vCornersEndIndex,
		int nSubPixIts

		);

void TrailPatch_1(char* kernelName,
		char * kernelFilename,
		int loopsize,
		clTrail* trails,
		int* nGoodTrails,
		clImage* Imageg,
		clImage* PreviousImg,
		clTemplate* mimOrigPatch,
		clTemplate* mimOrigPatchFmBackwrds,
		clvCorners* vCornerx,
		clvCorners* vCornerxPrev,
		int mnMaxSSD,
		int vCornerSize,
		int vPrevCornerSize,
		int mnHalfPatchSize

	);





void TestCallKernel(char* kernelName,char * kernelFilename);
