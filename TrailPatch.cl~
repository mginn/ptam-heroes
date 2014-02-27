
typedef struct clTrail    // This struct is used for initial correspondences of the first stereo pair.
{
	//MiniPatch mPatch;
	int irCurrentPos[2];
	int irInitialPos[2];
	bool  bFound;
}clTrail;

typedef struct clImage
{
	int image_size_x;
	int image_size_y;
	int image_stride;
	unsigned char im[307200];             

}clImage;

typedef struct clTemplate
{


	int image_size_x;
	int image_size_y;
	int image_stride;
	unsigned char im[81];                // The pyramid level pixels

}clTemplate;

typedef struct vCorners
{
	int vCon[2];
	
}vCorners;

int mag_squared(int x, int y) ;

int SSDAtPoint(__global clImage* Imageg, int ir_x, int ir_y, int mnHalfPatchSize,int my_size_x, int my_size_y,int stride,int mnMaxSSD,__global clTemplate* mimOrigPatch,int stride_mimOrigPatch,int gid);

void SampleFromImage(int irPos_x,int irPos_y, __global clImage* Imageg,int mnHalfPatchSize,	__global clTemplate* mimOrigPatchFmBackwrds,int im_stride, int mim_stride );

bool FindPatch(int *irpos_x,int *itpo_y ,__global clImage* Imageg, __global clTemplate* mimOrigPatch,	int nRange, __global vCorners* vConx,int mnMaxSSD,int vCornerSize,int mnHalfPatchSize,int gid);

__kernel void TrailPatch(
						int loopsize ,
						__global clTrail* trails,

						__global clImage* Imageg,
						
						__global clTemplate* mimOrigPatch,	
						__global vCorners* vCornerx,

						int mnMaxSSD,
						int vCornerSize,
						
						int mnHalfPatchSize,
						__global clTemplate* mimOrigPatchFmBackwrds,
						__global vCorners* vCornerxPrev,
						__global clImage* PreviousImg,
						int vPrevCornerSize,
						__global int* nGoodTrails
						
						 )	 	

{
	/*list<Trail>::iterator next = i; next++;

      Trail &trail = *i;*/
	int gid = get_global_id(0);
	if (gid >= loopsize)
    {   
        return; 
    }
	
	
	int irStart_x = trails[gid].irCurrentPos[0];
	int irStart_y = trails[gid].irCurrentPos[1];
	//ImageRef irEnd = irStart;
	int irEnd_x = irStart_x;
	int irEnd_y = irStart_y;

	

	///trails[gid].bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);//need to rewrite

	trails[gid].bFound = FindPatch(&irEnd_x,&irEnd_y, &Imageg[0],&mimOrigPatch[gid], 10, vCornerx,mnMaxSSD,vCornerSize,mnHalfPatchSize,gid); //same image is passed to each loop. so index 0	
	
			
	if(trails[gid].bFound)
	{
		// Also find backwards in a married-matches check
		SampleFromImage(irEnd_x, irEnd_y,&Imageg[0],mnHalfPatchSize,&mimOrigPatchFmBackwrds[gid],Imageg[0].image_stride,mimOrigPatchFmBackwrds[gid].image_stride);
	

	
	int irBackWardsFound_x = irEnd_x;
		int irBackWardsFound_y = irEnd_y;
		trails[gid].bFound = 	FindPatch(&irBackWardsFound_x,&irBackWardsFound_y, &PreviousImg[0],&mimOrigPatchFmBackwrds[gid], 10, vCornerxPrev,mnMaxSSD,vPrevCornerSize,mnHalfPatchSize,gid); //need to rewrite


	if(mag_squared(irBackWardsFound_x-irStart_x,irBackWardsFound_y-irStart_y)>2)
			trails[gid].bFound = false;

		trails[gid].irCurrentPos[0] = irEnd_x;
		trails[gid].irCurrentPos[1] = irEnd_y;
	
		nGoodTrails[gid]++;
	/*		if(gid==4)
	{
		
	
		//printf("kernel nSSD :%d\n",c);	
		printf("kernel trails[gid].bFound :%d\n",trails[gid].bFound);
		printf("kernel trails[gid].irCurrentPos[0] :%d\n",trails[gid].irCurrentPos[0]);	
		printf("kernel itrails[gid].irCurrentPos[1] :%d\n",trails[gid].irCurrentPos[1]);
		printf("kernel nGoodTrails[gid] :%d\n",nGoodTrails[gid]);
		printf("kernel magsqred :%d\n",mag_squared(irBackWardsFound_x-irStart_x,irBackWardsFound_y-irStart_y));				
		
		
		}*/
	}
	
	
} 


int mag_squared(int x, int y) 
{

	return (x*x) + (y*y);
}

void SampleFromImage(int irPos_x,int irPos_y, 
					__global clImage* Imageg,
					int mnHalfPatchSize,
					__global clTemplate* mimOrigPatchFmBackwrds,
					int im_stride, 
					int mim_stride )
{

	 
	bool check= irPos_x >=mnHalfPatchSize && irPos_y >=mnHalfPatchSize && irPos_x < (Imageg->image_size_x - mnHalfPatchSize) && irPos_y < (Imageg->image_size_y -mnHalfPatchSize) ;
	if(check)
	{
		int lft_x=irPos_x - (2 * mnHalfPatchSize + 1) / 2;
  		int lft_y=irPos_y - (2 * mnHalfPatchSize + 1) / 2;

		int m=0;
		  for (int i = lft_y; i < lft_y+mimOrigPatchFmBackwrds->image_size_x; i++)
			{
			  int n=0;
		  	  for (int j = lft_x;j < lft_x+mimOrigPatchFmBackwrds->image_size_y; j++)
		  	  {
		  		mimOrigPatchFmBackwrds->im[m*mim_stride+n]=Imageg->im[i*im_stride+j];
		  		n++;
		  	  }
		  	  m++;
			}
	}
		
		

}




bool FindPatch(int *irPos_x,int *irPos_y ,
		__global clImage* Imageg, 
		__global clTemplate* mimOrigPatch,
		int nRange, 
		__global vCorners* vConx,
		int mnMaxSSD,
		int vCornerSize,
		int mnHalfPatchSize,
		int gid
		)
{
	int irCenter_x = *irPos_x;
	int irCenter_y = *irPos_y;
	int irBest_x;
	int irBest_y;

	int nBestSSD = mnMaxSSD + 1;
	int irBBoxTL_x = *irPos_x - nRange;
	int irBBoxTL_y = *irPos_y - nRange;
	int irBBoxBR_x = *irPos_x +nRange;
	int irBBoxBR_y = *irPos_y +nRange;


	int index_begin=0;
	int index_end=vCornerSize;
	int index_i=0;
		for(index_i = index_begin; index_i!=index_end; index_i++)
			if(vConx[index_i].vCon[1]>=irBBoxTL_y ) break;
	int c=0;
	int k=index_i;
	for(int k=index_i; k!=index_end; k++)
	{
		if(vConx[k].vCon[0]<irBBoxTL_x||vConx[k].vCon[0]>irBBoxBR_x)
				continue;
		if(vConx[k].vCon[1]>irBBoxBR_y)
			break;

			
		int nSSD = SSDAtPoint(Imageg,
								vConx[k].vCon[0],
								vConx[k].vCon[1],
								mnHalfPatchSize,
								Imageg->image_size_x,
								Imageg->image_size_y,
								Imageg->image_stride,
								mnMaxSSD,
								mimOrigPatch,
								mimOrigPatch->image_stride,
								gid
								);
		c=nSSD;

		if(nSSD < nBestSSD)
		{
			irBest_x = vConx[k].vCon[0];
			irBest_y = vConx[k].vCon[1];
			nBestSSD = nSSD;
		}
	}

	
	
	
	
	if(nBestSSD < mnMaxSSD)
	{
		*irPos_x = irBest_x;
		*irPos_y = irBest_y;
		
		return true;
	}
	else
		return false;
}


int SSDAtPoint(__global clImage* Imageg, int ir_x, int ir_y, int mnHalfPatchSize,int my_size_x, int my_size_y,int stride,int mnMaxSSD,__global clTemplate* mimOrigPatch,int stride_mimOrigPatch,int gid)
{

bool check= ir_x >=mnHalfPatchSize && ir_y >=mnHalfPatchSize && ir_x < (my_size_x - mnHalfPatchSize) && ir_y < (my_size_y -mnHalfPatchSize) ;
  if(!check)
    return mnMaxSSD + 1;
  

	int irImgBase_x = ir_x - mnHalfPatchSize;
	int irImgBase_y = ir_y - mnHalfPatchSize;

  int nRows = mimOrigPatch->image_size_x;
  int nCols = mimOrigPatch->image_size_y;
 // byte *imagepointer;
  //byte *templatepointer;
  int nDiff;
  int nSumSqDiff = 0;

	
  
  for(int nRow = 0; nRow < nRows; nRow++)
    {
      	
      for(int nCol = 0; nCol < nCols; nCol++)
	{
	  
	  
		nDiff=Imageg->im[(irImgBase_y+nRow)*stride+(irImgBase_x+0)+nCol]-mimOrigPatch->im[nRow*stride_mimOrigPatch+0+nCol];

		nSumSqDiff += nDiff * nDiff;
		int img=Imageg->im[(irImgBase_y+nRow)*stride+(irImgBase_x+0)+nCol];
		int mim=mimOrigPatch->im[nRow*stride_mimOrigPatch+0+nCol];
		

	
	}
    }
  return nSumSqDiff;
}


