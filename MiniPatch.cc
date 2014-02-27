// Copyright 2008 Isis Innovation Limited
#include "MiniPatch.h"
#include "TimeCalculate.h"
using namespace CVD;
using namespace std;
#include<iostream>
#include <stdio.h>
#include <string>

// Scoring function
inline int MiniPatch::SSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir,int gid)
{
  if(!im.in_image_with_border(ir, mnHalfPatchSize))
    return mnMaxSSD + 1;
  ImageRef irImgBase = ir - ImageRef(mnHalfPatchSize, mnHalfPatchSize);
  int nRows = mimOrigPatch.size().y;
  int nCols = mimOrigPatch.size().x;
  byte *imagepointer;
  byte *templatepointer;
  int nDiff;
  int nSumSqDiff = 0;
  //cout<< "Loop itration at line 20 MiniPatch.cc nRows"<<nRows<<endl;
  //cout<< "Loop itration at line 24 MiniPatch.cc nCols"<<nCols<<endl;
  unsigned char *testimage=im.data();
  unsigned char *mnTemp=mimOrigPatch.data();
  int stride=im.row_stride();
  int strTemp=mimOrigPatch.row_stride();
  int my_size[2];
  my_size[0]=im.size().x;
  int img;
  int im_data;
  for(int nRow = 0; nRow < nRows; nRow++)
    {
      imagepointer = &im[irImgBase + ImageRef(0,nRow)];
      templatepointer = &mimOrigPatch[ImageRef(0,nRow)];
      for(int nCol = 0; nCol < nCols; nCol++)
	{
	  nDiff = imagepointer[nCol] - templatepointer[nCol];
	  nSumSqDiff += nDiff * nDiff;

	};
    };





  return nSumSqDiff;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners

bool MiniPatch::FindPatch(CVD::ImageRef &irPos, 
			  CVD::BasicImage<CVD::byte> &im, 
			  int nRange, 
			  vector<ImageRef> &vCorners,
			  int gid,
			  std::vector<int> *pvRowLUT
			  )
{
  ImageRef irCenter = irPos;
  ImageRef irBest;
  int nBestSSD = mnMaxSSD + 1;
  ImageRef irBBoxTL = irPos - ImageRef(nRange, nRange);
  ImageRef irBBoxBR = irPos + ImageRef(nRange, nRange);


  vector<ImageRef>::iterator i;
  if(!pvRowLUT)
    {
      for(i = vCorners.begin(); i!=vCorners.end(); i++)
 	if(i->y >= irBBoxTL.y) break;
    }
  else
    {
      int nTopRow = irBBoxTL.y;
      if(nTopRow < 0)
	nTopRow = 0;
      if(nTopRow >= (int) pvRowLUT->size())
	nTopRow = (int) pvRowLUT->size() - 1;
      i = vCorners.begin() + (*pvRowLUT)[nTopRow];
    }
//  cout<< "Loop itration at line 63 MiniPatch.cc vCorners.end()"<<std::distance(i,vCorners.end())<<endl;
  double i63=get_wall_time();
 // int d=vCorners.begin();
  int c=0;

  for(; i!=vCorners.end(); i++)
    {
      if(i->x < irBBoxTL.x  || i->x > irBBoxBR.x)
    	  continue;
      if(i->y > irBBoxBR.y)
    	  break;
      int nSSD = SSDAtPoint(im, *i,gid);
      c=nSSD;
      
      if(nSSD < nBestSSD)
	{
	  irBest = *i;
	  nBestSSD = nSSD;
	}
    }

  //cout<<"minipatrch prospective opencl candidate: "<<(i78-i63)<<endl;


  if(nBestSSD < mnMaxSSD)
    {
      irPos = irBest;

      return true;
    }
  else
    return false;
}

// Define the patch from an input image
void MiniPatch::SampleFromImage(ImageRef irPos, BasicImage<byte> &im)
{
  assert(im.in_image_with_border(irPos, mnHalfPatchSize));
  CVD::ImageRef irPatchSize( 2 * mnHalfPatchSize + 1 , 2 * mnHalfPatchSize + 1);
  mimOrigPatch.resize(irPatchSize);


  int lft_x=irPos.x - mimOrigPatch.size().x / 2;
  int lft_y=irPos.y - mimOrigPatch.size().y / 2;

  unsigned char* my_imagepnt=myImg.data();
  unsigned char* imagPtr=im.data();


  copy(im, mimOrigPatch, mimOrigPatch.size(), irPos - mimOrigPatch.size() / 2);

  /*int m=0;
  for (int i = lft_y; i < lft_y+9; i++)
    {
	  int n=0;
  	  for (int j = lft_x;j < lft_x+9; j++)
  	  {
  		my_imagepnt[m*myImg.row_stride()+n]=imagPtr[i*im.row_stride()+j];
  		n++;
  	  }
  	  m++;
    }









/*

  FILE *fpi = fopen("input_image.txt", "w");
  for (int i = lft_y; i < lft_y+9; i++)
  {
	  fprintf(fpi,"\nrow %2d: ", i);
	  for (int j = lft_x; j < lft_x+9 ; j++)
	  {
		  fprintf(fpi," %3d ", myImg[i][j]);
	  }
  }
  fclose(fpi);
*/


}

// Static members
int MiniPatch::mnHalfPatchSize = 4;
int MiniPatch::mnRange = 10;
int MiniPatch::mnMaxSSD = 9999;













