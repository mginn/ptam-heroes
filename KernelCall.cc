/*
 * KernelCall.cc
 *
 *  Created on: Jan 9, 2014
 *      Author: mainul
 */

#include <stdio.h>
#include <string.h>
#include <string>
#include "KernelCall.h"
#include "TimeCalculate.h"
#include <iostream>
using namespace std;

cl_context cxGPUContext;        // OpenCL context
cl_command_queue cqCommandQueue;// OpenCL command que
cl_platform_id cpPlatform;      // OpenCL platform
cl_device_id cdDevice;          // OpenCL device
cl_program cpProgram;           // OpenCL program
cl_kernel ckKernel;             // OpenCL kernel
cl_mem cmDevSrcA;               // OpenCL device source buffer A
cl_mem cmDevSrcB;               // OpenCL device source buffer B
cl_mem cmDevDst;                // OpenCL device destination buffer
size_t szGlobalWorkSize;        // 1D var for Total # of work items
size_t szLocalWorkSize;		// 1D var for # of work items in the work group
size_t szParmDataBytes;		// Byte size of context information
size_t szKernelLength;		// Byte size of kernel code
cl_int ciErr1, ciErr2;		// Error code var
char* cPathAndName = NULL;      // var for full paths to data, src, etc.
char* cSourceCL = NULL;         // Buffer to hold source for compilation

int convertToString(char *filename, std::string& s)
{
    size_t size;
    char*  str;

    std::fstream f(filename, (std::fstream::in | std::fstream::binary));

    if(f.is_open())
    {
        size_t fileSize;
        f.seekg(0, std::fstream::end);
        size = fileSize = (size_t)f.tellg();
        f.seekg(0, std::fstream::beg);

        str = new char[size+1];
        if(!str)
        {
            f.close();
            return 1;
        }

        f.read(str, fileSize);
        f.close();
        str[size] = '\0';

        s = str;
        delete[] str;
        return 0;
    }
    printf("Error: Failed to open file %s\n", filename);
    return 1;
}

void TestCallKernel(char* kernelName,char * kernelFilename)
{

	int i;
		const int LIST_SIZE = 2560;
		int *A = (int*)malloc(sizeof(int)*LIST_SIZE);
		int *B = (int*)malloc(sizeof(int)*LIST_SIZE);
		int *Golden=(int*)malloc(sizeof(int)*LIST_SIZE);

		for(i = 0; i < LIST_SIZE; i++) {
			//A[i] = -1;
			B[i] = 1;
		}

		ciErr1=clGetPlatformIDs(1, &cpPlatform, NULL);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clGetPlatformID, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		printf("*** Got platform\n");
		printf("*** %d\n",cpPlatform);

		ciErr1 = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, 1, &cdDevice, NULL);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clGetDeviceIDs, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		printf("*** Got device\n");

		cxGPUContext = clCreateContext(0, 1, &cdDevice, NULL, NULL, &ciErr1);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clCreateContext, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		printf("*** Got context\n");



		cqCommandQueue = clCreateCommandQueue(cxGPUContext, cdDevice, 0, &ciErr1);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clCreateCommandQueue, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		printf("*** Got commandqueue\n");

		//const char * filename  = "TestKernel.cl";
		std::string  sourceStr;
		ciErr1 = convertToString(kernelFilename, sourceStr);
		if(ciErr1 != CL_SUCCESS) {
			printf("Error in convertToString, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		const char * source    = sourceStr.c_str();
		size_t sourceSize[]    = { strlen(source) };


		cpProgram = clCreateProgramWithSource(cxGPUContext, 1, &source, sourceSize, &ciErr1);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clCreateProgramWithSource, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		printf("*** Got createprogramwithsource\n");

		ciErr1 =  clBuildProgram(cpProgram, 1, &cdDevice, NULL, NULL, NULL);


		printf("*** Got buildprogram\n");


		ckKernel = clCreateKernel(cpProgram, kernelName, &ciErr1);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clCreateKernel, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}

		printf("*** Got createkernel\n");

		// Create memory buffers on the device for each vector
		/*------create buffer------------------- */
		// cl_mem a_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, LIST_SIZE * sizeof(int), NULL, &ciErr1);

		cl_mem a_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_ALLOC_HOST_PTR, LIST_SIZE * sizeof(int), NULL, &ciErr1);

		cl_int* inputMap = (cl_int*) clEnqueueMapBuffer(cqCommandQueue, a_mem_obj, CL_TRUE,CL_MAP_READ|CL_MAP_WRITE, 0, sizeof(cl_int) * 		LIST_SIZE, 0, NULL, NULL, &ciErr2);
		ciErr1 |= ciErr2;

		//memcpy (inputMap,A,sizeof(int)*LIST_SIZE);
		for(i = 0; i < LIST_SIZE; i++) {
			inputMap[i]  = -1;

		}

		clEnqueueUnmapMemObject(cqCommandQueue, a_mem_obj, inputMap, 0, NULL, NULL);

		ciErr1 |= ciErr2;

		cl_mem b_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, LIST_SIZE * sizeof(int), NULL,&ciErr2);
		ciErr1 |= ciErr2;
		cl_mem c_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_WRITE_ONLY,LIST_SIZE * sizeof(int), NULL, &ciErr2);

		//cl_mem c_mem_obj=clCreateBuffer(cxGPUContext, CL_MEM_ALLOC_HOST_PTR, LIST_SIZE * sizeof(int), NULL, &ciErr1);

		ciErr1 |= ciErr2;
		// Copy the lists A and B to their respective memory buffers

		// ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, a_mem_obj, CL_TRUE, 0, LIST_SIZE * sizeof(int), A, 0, NULL, NULL);
		ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, b_mem_obj, CL_TRUE, 0, LIST_SIZE * sizeof(int), B, 0, NULL, NULL);

		ciErr1 = clSetKernelArg(ckKernel, 0, sizeof(cl_mem), (void*)&a_mem_obj);
		ciErr1 |= clSetKernelArg(ckKernel, 1, sizeof(cl_mem), (void*)&b_mem_obj);
		ciErr1 |= clSetKernelArg(ckKernel, 2, sizeof(cl_mem), (void*)&c_mem_obj);
		ciErr1 |= clSetKernelArg(ckKernel, 3, sizeof(cl_int), (void*)&LIST_SIZE);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clSetKernelArg, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		printf("*** Got setkernelarg\n");
		// Execute the OpenCL kernel on the list
		size_t global_item_size = LIST_SIZE; // Process the entire lists
		size_t local_item_size = 256;

		cl_int q;
		q = clEnqueueNDRangeKernel(cqCommandQueue, ckKernel, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);
		if (q != CL_SUCCESS) {
			printf("Error in clEnqueueNDRangeKernel, Line %u in file %s !!!\n\n", __LINE__, __FILE__);
			//printf("%d",q);
		}
		clFinish(cqCommandQueue);
		printf("*** Got enqueuendrangekernel\n");
		// Read the memory buffer C on the device to the local variable C
		//int *C = (int*)malloc(sizeof(int)*LIST_SIZE);
		//int *C = (cl_int*) clEnqueueMapBuffer(cqCommandQueue, c_mem_obj, CL_TRUE,CL_MAP_READ, 0, sizeof(cl_int) * 		LIST_SIZE, 0, NULL, NULL, &ciErr2);

		cl_int p;
		//p = clEnqueueReadBuffer(cqCommandQueue, c_mem_obj, CL_TRUE, 0, sizeof(cl_int) * LIST_SIZE, C, 0, NULL, NULL);
		/*if (p != CL_SUCCESS) {
			printf("Error in clEnqueueReadBuffer, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}*/
		printf("*** Got enqueuereadbuffer\n");

		// Display the result to the screen
		/*for(i = 0; i < 10; i++)
			printf("%d + %d = %d\n", inputMap[i], B[i], C[i]);

		clEnqueueUnmapMemObject(cqCommandQueue, c_mem_obj, C, 0, NULL, NULL);*/



		printf("*** Got VectorAddHost\n");
		/*for(int idx = 0; idx < 10; idx++) {
			printf("%d ", Golden[idx]);
		}*/

		bool bMatch = true;
		/*for(int idx = 0; idx < 10; idx++) {
			if(Golden[idx] != C[idx]) {
				bMatch = false;
				printf("idx %d doesn't match. %d vs %d", idx, Golden[idx], C[idx]);
				break;
			}
		}
		printf("*** Got checkMatch\n");*/

		// Clean up
		ciErr1 = clFlush(cqCommandQueue);
		ciErr1 = clFinish(cqCommandQueue);
		ciErr1 = clReleaseKernel(ckKernel);
		ciErr1 = clReleaseProgram(cpProgram);
		ciErr1 = clReleaseMemObject(a_mem_obj);
		ciErr1 = clReleaseMemObject(b_mem_obj);
		ciErr1 = clReleaseMemObject(c_mem_obj);
		ciErr1 = clReleaseCommandQueue(cqCommandQueue);
		ciErr1 = clReleaseContext(cxGPUContext);
		free(A);
		free(B);
		//free(C);
}

int SetContext()
{
	ciErr1=clGetPlatformIDs(1, &cpPlatform, NULL);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clGetPlatformID, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}


	ciErr1 = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, 1, &cdDevice, NULL);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clGetDeviceIDs, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
//	printf("*** Got device\n");
	//printf("*** Got device*****************************************\n");
	//printf("************** %d********************************\n",cdDevice);

	cxGPUContext = clCreateContext(0, 1, &cdDevice, NULL, NULL, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateContext, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	else
	printf("*** Got context\n");



	cqCommandQueue = clCreateCommandQueue(cxGPUContext, cdDevice, 0, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateCommandQueue, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	else
	printf("*** Got commandqueue\n");
	return 1;

}
int SearchForPointsKernel(char* kernelName,
		char * kernelFilename,
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
	)
{



#if(1) //setting device and kernel
	ciErr1=clGetPlatformIDs(1, &cpPlatform, NULL);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clGetPlatformID, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}


	ciErr1 = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, 1, &cdDevice, NULL);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clGetDeviceIDs, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	printf("*** Got device\n");
	//printf("*** Got device*****************************************\n");
	//printf("************** %d********************************\n",cdDevice);

	cxGPUContext = clCreateContext(0, 1, &cdDevice, NULL, NULL, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateContext, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	printf("*** Got context\n");



	cqCommandQueue = clCreateCommandQueue(cxGPUContext, cdDevice, 0, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateCommandQueue, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	printf("*** Got commandqueue\n");
	//const char * filename  = "TestKernel.cl";
	std::string  sourceStr;
	ciErr1 = convertToString(kernelFilename, sourceStr);
	if(ciErr1 != CL_SUCCESS) {
		printf("Error in convertToString, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	const char * source    = sourceStr.c_str();
	size_t sourceSize[]    = { strlen(source) };


	cpProgram = clCreateProgramWithSource(cxGPUContext, 1, &source, sourceSize, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateProgramWithSource, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	printf("*** Got createprogramwithsource\n");
	//cout<<source<<endl;


	ciErr1 =  clBuildProgram(cpProgram, 1, &cdDevice, NULL, NULL, NULL);


	printf("*** Got buildprogram\n");

	cout<<kernelName<<endl;

	ckKernel = clCreateKernel(cpProgram, kernelName, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateKernel, Line %u in file %s error NO: %d!!!\n\n", __LINE__, __FILE__,ciErr1);

	}



#endif


	cl_mem TD_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, vTDVectorSize * sizeof(clTrackerData), NULL,&ciErr2);
	cl_mem nFound_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, vTDVectorSize * sizeof(int), NULL,&ciErr2);
	cl_mem Finder_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, vTDVectorSize * sizeof(clFinder), NULL,&ciErr2);
	//cl_mem manMeasAttempted_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, LEVELSs * sizeof(int), NULL,&ciErr2);
	cl_mem Image_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, vTDVectorSize * sizeof(clImage), NULL,&ciErr2);
	cl_mem Level_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, vTDVectorSize * sizeof(clLevel), NULL,&ciErr2);
	cl_mem mimTempl_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, vTDVectorSize * sizeof(clImage), NULL,&ciErr2);
	cl_mem vCornerEndIndx_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, vTDVectorSize * sizeof(int), NULL,&ciErr2);


	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, TD_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(clTrackerData), TD, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, nFound_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(int), nFound, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, Finder_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(clFinder), Finder, 0, NULL, NULL);
	//ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, manMeasAttempted_mem_obj, CL_TRUE, 0, LEVELSs * sizeof(int), manMeasAttempted, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, Image_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(clImage), Imageg, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, Level_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(clLevel), Levelg, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, mimTempl_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(clImage), mimTemplate, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, vCornerEndIndx_mem_obj, CL_TRUE, 0, vTDVectorSize * sizeof(int), mimTemplate, 0, NULL, NULL);

	ciErr1 = clSetKernelArg(ckKernel, 0, sizeof(cl_int), (void*)&nRange);
	ciErr1 = clSetKernelArg(ckKernel, 1, sizeof(cl_mem), (void*)&TD_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 2, sizeof(cl_mem), (void*)&nFound_mem_obj);

	ciErr1 |= clSetKernelArg(ckKernel, 3, sizeof(cl_int), (void*)&vTDVectorSize);
	ciErr1 |= clSetKernelArg(ckKernel, 4, sizeof(cl_mem), (void*)&Finder_mem_obj);
	//ciErr1 |= clSetKernelArg(ckKernel, 5, sizeof(cl_mem), (void*)&manMeasAttempted_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 5, sizeof(cl_mem), (void*)&Image_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 6, sizeof(cl_mem), (void*)&Level_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 7, sizeof(cl_mem), (void*)&mimTempl_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 8, sizeof(cl_mem), (void*)&vCornerEndIndx_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 9, sizeof(cl_int), (void*)&nSubPixIts);

	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clSetKernelArg, Line %u in file %s %d!!!\n\n", __LINE__, __FILE__,ciErr1);
	}

	//cout<<"vecor size: "<<vTDVectorSize<<endl;


	int temp_size=vTDVectorSize/32;
	size_t global_item_size = 32*(temp_size+1); // Process the entire lists
	size_t local_item_size = 32;

	cout<<"global item size: "<<global_item_size<<endl;

	cl_int q;
	double i442=get_wall_time();
	q = clEnqueueNDRangeKernel(cqCommandQueue, ckKernel, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);
	clFinish(cqCommandQueue);
	double i445=get_wall_time();
	cout<<"kernel execution time.........."<<(i445-i442)<<endl;

	if (q != CL_SUCCESS) {
		printf("Error in clEnqueueNDRangeKernel, Line %u in file %s  %d!!!\n\n", __LINE__, __FILE__,q);

	}
	printf("*** Got enqueuendrangekernel\n");

	cl_int p;

	p = clEnqueueReadBuffer(cqCommandQueue, TD_mem_obj, CL_TRUE, 0, sizeof(clTrackerData) * vTDVectorSize, TD, 0, NULL, NULL);
	p = clEnqueueReadBuffer(cqCommandQueue, Finder_mem_obj, CL_TRUE, 0, sizeof(clFinder) * vTDVectorSize, Finder, 0, NULL, NULL);


	// Clean up
	ciErr1 = clFlush(cqCommandQueue);
	ciErr1 = clFinish(cqCommandQueue);
	ciErr1 = clReleaseKernel(ckKernel);
	ciErr1 = clReleaseProgram(cpProgram);
	ciErr1 = clReleaseMemObject(TD_mem_obj);
	ciErr1 = clReleaseMemObject(nFound_mem_obj);
	//ciErr1 = clReleaseMemObject(c_mem_obj);
	ciErr1 = clReleaseCommandQueue(cqCommandQueue);
	ciErr1 = clReleaseContext(cxGPUContext);


	return 1;
}


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

	)
{
#if(1) //setting device and kernel
/*//cl_uint platforms;
	ciErr1=clGetPlatformIDs(1, &cpPlatform, NULL);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clGetPlatformID, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}


		ciErr1 = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, 1, &cdDevice, NULL);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clGetDeviceIDs, Line %u in file %s %d!!!\n\n", __LINE__, __FILE__,ciErr1);

		}
		else
	printf("*** Got device\n");
		//printf("*** Got device*****************************************\n");
		//printf("************** %d********************************\n",cdDevice);

		cxGPUContext = clCreateContext(0, 1, &cdDevice, NULL, NULL, &ciErr1);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clCreateContext, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		else
		printf("*** Got context\n");



		cqCommandQueue = clCreateCommandQueue(cxGPUContext, cdDevice, 0, &ciErr1);
		if (ciErr1 != CL_SUCCESS) {
			printf("Error in clCreateCommandQueue, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

		}
		else
		printf("*** Got commandqueue\n");


*/


	std::string  sourceStr;
	ciErr1 = convertToString(kernelFilename, sourceStr);
	if(ciErr1 != CL_SUCCESS) {
		printf("Error in convertToString, Line %u in file %s !!!\n\n", __LINE__, __FILE__);

	}
	const char * source    = sourceStr.c_str();
	size_t sourceSize[]    = { strlen(source) };


	cpProgram = clCreateProgramWithSource(cxGPUContext, 1, &source, sourceSize, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateProgramWithSource, Line %u in file %s %d !!!\n\n", __LINE__, __FILE__,ciErr1);

	}
	else
//	printf("*** Got createprogramwithsource\n");
	//cout<<source<<endl;


	ciErr1 =  clBuildProgram(cpProgram, 1, &cdDevice, NULL, NULL, NULL);


	//printf("*** Got buildprogram\n");

	//cout<<kernelName<<endl;

	ckKernel = clCreateKernel(cpProgram, kernelName, &ciErr1);
	if (ciErr1 != CL_SUCCESS) {
		printf("Error in clCreateKernel, Line %u in file %s error NO: %d!!!\n\n", __LINE__, __FILE__,ciErr1);

	}



#endif


/*
	printf("CPU loopsize :%d\n",loopsize);
	printf("CPU trails[gid].bFound :%d\n",trails[4].bFound);
	printf("CPU trails[gid].irCurrentPos[0] :%d\n",trails[4].irCurrentPos[0]);
	printf("CPU trails[gid].irCurrentPos[1] :%d\n",trails[4].irCurrentPos[1]);
	printf("CPU nGoodTrails[gid] :%d\n",nGoodTrails[4]);
	printf("CPU imnMaxSSD :%d\n",mnMaxSSD);
	printf("CPU vCornerSize :%d\n",vCornerSize);
	printf("CPU mnHalfPatchSize :%d\n",mnHalfPatchSize);


	printf("CPU vCornerxPrev[gid].[1] :%d\n",vCornerxPrev[4].vCon[1]);
	printf("CPU vPrevCornerSize[gid] :%d\n",vPrevCornerSize);
	printf("CPU vCornerx[gid] :%d\n",vCornerx[4].vCon[1]);

*/




	cl_mem Trail_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, loopsize * sizeof(clTrail), NULL,&ciErr2);
	cl_mem CurImage_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY,  sizeof(clImage), NULL,&ciErr2);
	size_t image_size=loopsize*sizeof(clTemplate);
	//cout<<"Image size......."<<image_size<<endl;
	//cout<<"loop size......."<<loopsize<<endl;

	//cout<<"Image size......."<<sizeof(clTemplate)<<endl;

	cl_mem mimOrg_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY,  image_size, NULL,&ciErr2);


	//cl_mem mimImage_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, loopsize * sizeof(clImage), NULL,&ciErr2);
	cl_mem mimBackImage_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY, loopsize * sizeof(clTemplate), NULL,&ciErr2);
	cl_mem PrevImage_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY,  sizeof(clImage), NULL,&ciErr2);
	cl_mem vCorn_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, vCornerSize * sizeof(clvCorners), NULL,&ciErr2);
	cl_mem vPrevCorn_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, vPrevCornerSize * sizeof(clvCorners), NULL,&ciErr2);
	cl_mem knGTrails_mem_obj = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE, loopsize * sizeof(int), NULL,&ciErr2);

	if (ciErr2 != CL_SUCCESS) {
						printf("Error in creating memory objects, Line %u in file %s error NO: %d!!!\n\n", __LINE__, __FILE__,ciErr2);

					}

	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, Trail_mem_obj, CL_TRUE, 0, loopsize * sizeof(clTrail), trails, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, CurImage_mem_obj, CL_TRUE, 0,  sizeof(clImage), Imageg, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, mimOrg_mem_obj, CL_TRUE, 0,  sizeof(clTemplate)*loopsize, mimOrigPatch, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, PrevImage_mem_obj, CL_TRUE, 0,  sizeof(clImage), PreviousImg, 0, NULL, NULL);
	//ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, mimImage_mem_obj, CL_TRUE, 0, loopsize * sizeof(clImage), mimOrigPatch, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, mimBackImage_mem_obj, CL_TRUE, 0, loopsize * sizeof(clTemplate), mimOrigPatchFmBackwrds, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, vCorn_mem_obj, CL_TRUE, 0, vCornerSize * sizeof(clvCorners), vCornerx, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, vPrevCorn_mem_obj, CL_TRUE, 0, vPrevCornerSize * sizeof(clvCorners), vCornerxPrev, 0, NULL, NULL);
	ciErr1 = clEnqueueWriteBuffer(cqCommandQueue, knGTrails_mem_obj, CL_TRUE, 0, loopsize * sizeof(int), nGoodTrails, 0, NULL, NULL);

	if (ciErr1 != CL_SUCCESS) {
				printf("Error in creating buffers, Line %u in file %s error NO: %d!!!\n\n", __LINE__, __FILE__,ciErr1);

			}

	ciErr1 = clSetKernelArg(ckKernel, 0, sizeof(cl_int), (void*)&loopsize);
	ciErr1 = clSetKernelArg(ckKernel, 1, sizeof(cl_mem), (void*)&Trail_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 2, sizeof(cl_mem), (void*)&CurImage_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 3, sizeof(cl_mem), (void*)&mimOrg_mem_obj);
//	ciErr1 |= clSetKernelArg(ckKernel, 3, sizeof(cl_mem), (void*)&mimImage_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 4, sizeof(cl_mem), (void*)&vCorn_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 5, sizeof(cl_int), (void*)&mnMaxSSD);
	ciErr1 |= clSetKernelArg(ckKernel, 6, sizeof(cl_int), (void*)&vCornerSize);
	ciErr1 |= clSetKernelArg(ckKernel, 7, sizeof(cl_int), (void*)&mnHalfPatchSize);
	ciErr1 |= clSetKernelArg(ckKernel, 8, sizeof(cl_mem), (void*)&mimBackImage_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 9, sizeof(cl_mem), (void*)&vPrevCorn_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 10, sizeof(cl_mem), (void*)&PrevImage_mem_obj);
	ciErr1 |= clSetKernelArg(ckKernel, 11, sizeof(cl_int), (void*)&vPrevCornerSize);
	ciErr1 |= clSetKernelArg(ckKernel, 12, sizeof(cl_mem), (void*)&knGTrails_mem_obj);

	if (ciErr1 != CL_SUCCESS) {
			printf("Error in creating args, Line %u in file %s error NO: %d!!!\n\n", __LINE__, __FILE__,ciErr1);

		}

	int temp_size=loopsize/32;
	size_t global_item_size = 32*(temp_size+1); // Process the entire lists
	size_t local_item_size = 32;

	//cout<<"global item size: "<<global_item_size<<endl;

	//printf("CPU mimOrigPatchFmBackwrds im[0] :%d\n",mimOrigPatchFmBackwrds[4].im[112]);
	//	printf("CPU PreviousImg[gid].[1] :%d\n",PreviousImg->im[355]);
	/*printf("CPU vConx[k].vCon[0] im[0] :%d\n",vCornerx[4].vCon[0]);

	printf("CPU vConx[k].vCon[1][gid].[1] :%d\n",vCornerx[4].vCon[1]);

	printf("CPU Imageg->image_size_x[gid].[1] :%d\n",Imageg->image_size_x);
	printf("CPU Imageg->image_size_y :%d\n",Imageg->image_size_y);
	printf("CPU Imageg[gid] :%d\n",Imageg->im[428]);
	printf("CPU Imageg->image_stride :%d\n",Imageg->image_stride);
	printf("CPU mimOrigPatch[gid].[1] :%d\n",mimOrigPatch[4].im[9]);
	printf("CPU mimOrigPatch image_size_x :%d\n",mimOrigPatch[4].image_size_x);
	printf("CPU mimOrigPatch->image_stride :%d\n",mimOrigPatch->image_stride);
	printf("CPU mnMaxSSD :%d\n",mnMaxSSD);
	printf("CPU mnHalfPatchSize :%d\n",mnHalfPatchSize);

*/
	cl_int q;
	double i547=get_wall_time();
	q = clEnqueueNDRangeKernel(cqCommandQueue, ckKernel, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);

	clFinish(cqCommandQueue);
	double i550=get_wall_time();
	cout<<"kernel execution time.........."<<(i550-i547)<<endl;
	cl_int p;

	p = clEnqueueReadBuffer(cqCommandQueue, Trail_mem_obj, CL_TRUE, 0, sizeof(clTrail) * loopsize, trails, 0, NULL, NULL);
	p = clEnqueueReadBuffer(cqCommandQueue, knGTrails_mem_obj, CL_TRUE, 0, sizeof(int) * loopsize, nGoodTrails, 0, NULL, NULL);

	/*int h=0;
	for(int j=0;j<loopsize;j++)
	{
		h=h+nGoodTrails[j];
	}*/
	//cout<<"kernel ntrails :"<<h<<endl;
	//printf("*** from kernel\n");

	///Clean up

	ciErr1 = clFlush(cqCommandQueue);
	ciErr1 = clFinish(cqCommandQueue);
	ciErr1 = clReleaseKernel(ckKernel);
	ciErr1 = clReleaseProgram(cpProgram);
	ciErr1 = clReleaseMemObject(Trail_mem_obj);
	ciErr1 = clReleaseMemObject(CurImage_mem_obj);
	ciErr1 = clReleaseMemObject(mimOrg_mem_obj);
	ciErr1 = clReleaseMemObject(vCorn_mem_obj);
	ciErr1 = clReleaseMemObject(mimBackImage_mem_obj);
	ciErr1 = clReleaseMemObject(vPrevCorn_mem_obj);
	ciErr1 = clReleaseMemObject(PrevImage_mem_obj);
	ciErr1 = clReleaseMemObject(knGTrails_mem_obj);




}
