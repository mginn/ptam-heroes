// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "TrackerData.h"

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <fstream>
#include <fcntl.h>
#include <sys/time.h>
//added by mainul

#include "TimeCalculate.h"
#include <ncurses.h>
#include "KernelCall.h"
//added to include space press
//
#include <sstream>
#include <stdlib.h>
#include <ostream>
#include <string>

using namespace CVD;
using namespace std;
using namespace GVars3;
int globalCounterForKernelCall=0;


// The constructor mostly sets up internal reference variables
// to the other classes..
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm) : 
  mMap(m),
  mMapMaker(mm),
  mCamera(c),
  mRelocaliser(mMap, mCamera),
  mirSize(irVideoSize)
{
  mCurrentKF.bFixed = false;
  //GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  //GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  //GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;

  // Most of the initialisation is done in Reset()
  Reset();
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{
  mbDidCoarse = false;
  mbUserPressedSpacebar = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = false;
  
  // Tell the MapMaker to reset itself.. 
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.
  mMapMaker.RequestReset();
  while(!mMapMaker.ResetDone())
#ifndef WIN32
	  usleep(10);
#else
	  Sleep(1);
#endif
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker whether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::TrackFrame(Image<byte> &imFrame, bool bDraw)
{
  mbDraw = bDraw;
  mMessageForUser.str("");   // Wipe the user message clean
  
  // Take the input video image, and convert it into the tracker's keyframe struct
  // This does things like generate the image pyramid and find FAST corners
  double cl1=get_wall_time();
  mCurrentKF.mMeasurements.clear();
 /* double cl2=get_wall_time();
 // cout<<"TIme to clear "<<endl;//mnl>>print the time
 // cout<<(cl2-cl1)<<endl;


  double mk1=get_wall_time();*/
  mCurrentKF.MakeKeyFrame_Lite(imFrame); //mnl is important.need to call always.not good for opencl
  double mk2=get_wall_time();
 // cout<<"TIme to MakeKeyFrame_Lite Map"<<endl;//mnl>>print the time
 // cout<<(mk2-mk1)<<endl;


  // Update the small images for the rotation estimator
  double rte1=get_wall_time();
  static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = *gvnUseSBI;
  if(!mpSBIThisFrame)
    {
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
      mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  else
    {
      delete  mpSBILastFrame;
      mpSBILastFrame = mpSBIThisFrame;
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  
  // From now on we only use the keyframe struct!
  mnFrame++;
  //cout<< "Loop teeation at line 144 tracker.cc mCurrentKF.aLevels[0].vCorners.size()"<<mCurrentKF.aLevels[0].vCorners.size()<<endl;
  if(mbDraw) //mnl>>this draw frames
    {
      glDrawPixels(mCurrentKF.aLevels[0].im);
      if(GV2.GetInt("Tracker.DrawFASTCorners",0, SILENT))
	{
	  glColor3f(1,0,1);  glPointSize(1); glBegin(GL_POINTS);
	  for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCorners.size(); i++) 
	    glVertex(mCurrentKF.aLevels[0].vCorners[i]);
	  glEnd();
	}
    }
  double rte2=get_wall_time();
  //cout<<"TIme to rotation map and some if else before calling applymotion model and others "<<endl;//mnl>>print the time
  //cout<<(rte2-rte1)<<endl;
  // Decide what to do - if there is a map, try to track the map ...
  if(mMap.IsGood())
  {
	  if(mnLostFrames < 3)  // .. but only if we're not lost!
	  {
		  if(mbUseSBIInit)
		  {
			  double mk3=get_wall_time();
			  CalcSBIRotation();
			  double mk4=get_wall_time();
			  //cout<<"TIme to CalcSBIRotation Map"<<endl;//mnl>>print the time
			  //cout<<(mk4-mk3);

		  }

		  double i1=get_wall_time();//mnl>>chekce the time

		  ApplyMotionModel();       //
		  double i2=get_wall_time();//mnl>>chekce the timeostringstream os ;
		  ostringstream os ;
		  //os << (i2-i1);
		 // cout<<"TIme to Apply motion model"<<os.str()<<endl;//mnl>>print the time
		  ostringstream os1;
		  double i3=get_wall_time();//mnl>>chekce the time
		  TrackMap();               //  These three lines do the main tracking work.
		  double i4=get_wall_time();


		  //cout<<os1.str();
		 // cout<<"TIme to TrackMap Map"<<endl;//mnl>>print the time
		 // cout<<(i4-i3);
//
		  double i5=get_wall_time(); //mnl>>chekce the time
		  UpdateMotionModel();      //
		  double i6=get_wall_time();

		 // cout<<"TIme to UpdateMotionModel Map"<<endl;//mnl>>print the time
		 // cout<<(i6-i5);

		  double mk5=get_wall_time();
		  AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
		  double mk6=get_wall_time();
		 // cout<<"TIme to AssessTrackingQuality Map"<<endl;//mnl>>print the time
		 // cout<<(mk6-mk5);

		  FILE *fpi = fopen("messg_f_usr.txt", "w");

		  	  fprintf(fpi,"\nTracking Map, quality: ");
		  	  if(mTrackingQuality == GOOD)fprintf(fpi,"good");
		  	  if(mTrackingQuality == DODGY) fprintf(fpi,"poor ");
		  	  if(mTrackingQuality == BAD)   fprintf(fpi,"bad ");

		  	  fclose(fpi);
//edited the following portion so that output can be written to file.


/*
		  { // Provide some feedback for the user:
			  mMessageForUser << "Tracking Map, quality ";
			  if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
			  if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
			  if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
			  mMessageForUser << " Found:";
			  for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
			  //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
			  mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
		  }

		*/
		  // Heuristics to check if a key-frame should be added to the map:
		  if(mTrackingQuality == GOOD &&
				  mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
				  mnFrame - mnLastKeyFrameDropped > 20  &&
				  mMapMaker.QueueSize() < 3)
		  {
			  mMessageForUser << " Adding key-frame.";

			  double mk7=get_wall_time();
			  AddNewKeyFrame();
			  double mk8=get_wall_time();
			//  cout<<"TIme to AddNewKeyFrame Map"<<endl;//mnl>>print the time
			//  cout<<(mk8-mk7);
		  };
	  }
	  else  // what if there is a map, but tracking has been lost?
	  {
		 // cout<< "inside else1..."<<endl;
		  mMessageForUser << "** Attempting recovery **.";
		  if(AttemptRecovery())
		  {
			//  cout<< "inside else2..."<<endl;
			  TrackMap();
			  AssessTrackingQuality();
		  }
	  }
	  if(mbDraw)
	  {
		  //cout<<"another if at line 227"<<endl;
		  RenderGrid();

	  }
  }
  else // If there is no map, try to make one.
  {
	  double mk9=get_wall_time();
	  TrackForInitialMap();
	  double mk10=get_wall_time();
	  //cout<<"TIme to TrackForInitialMap Map"<<endl;//mnl>>print the time
	  //cout<<(mk10-mk9);
  }
int cx=0;
  // GUI interface
  double wh1=get_wall_time();
 /* while(!mvQueuedCommands.empty())
    {
      GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
      mvQueuedCommands.erase(mvQueuedCommands.begin());
      cx++;
    }*/
  double wh2=get_wall_time();
//  cout<<"Loop itration at line 253 tracker.cc cx"<< cx<<endl;//mnl>>print the time
  //cout<<(wh2-wh1);
};

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
  bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
  if(!bRelocGood)
    return false;
  
  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void Tracker::RenderGrid()
{
  // The colour of the ref grid shows if the coarse stage of tracking was used
  // (it's turned off when the camera is sitting still to reduce jitter.)
  if(mbDidCoarse)
    glColor4f(.0, 0.5, .0, 0.6);
  else
    glColor4f(0,0,0,0.6);
  
  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  int nHalfCells = 8;
  int nTot = nHalfCells * 2 + 1;
  Image<Vector<2> >  imVertices(ImageRef(nTot,nTot));

 // cout<< "Loop teeation at line 299 tracker.cc nTOT"<<nTot<<endl;
  for(int i=0; i<nTot; i++)
	  for(int j=0; j<nTot; j++)
	  {
		  Vector<3> v3;
		  v3[0] = (i - nHalfCells) * 0.1;
		  v3[1] = (j - nHalfCells) * 0.1;
		  v3[2] = 0.0;
		  Vector<3> v3Cam = mse3CamFromWorld * v3;
		  if(v3Cam[2] < 0.001)
			  v3Cam[2] = 0.001;
		  imVertices[i][j] = mCamera.Project(project(v3Cam));
	  }
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
  {
	  glBegin(GL_LINE_STRIP);
	  for(int j=0; j<nTot; j++)
		  glVertex(imVertices[i][j]);
	  glEnd();

	  glBegin(GL_LINE_STRIP);
	  for(int j=0; j<nTot; j++)
		  glVertex(imVertices[j][i]);
	  glEnd();
  };

  glLineWidth(1);
  glColor3f(1,0,0);
}

// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
/*void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="Reset")
    {
      Reset();
      return;
    }

  // KeyPress commands are issued by GLWindow
  if(sCommand=="KeyPress")
    {
      if(sParams == "Space")
	{
	  mbUserPressedSpacebar = true;
	}
      else if(sParams == "r")
	{
	  Reset();
	}
      else if(sParams == "q" || sParams == "Escape")
	{
	  GUI.ParseLine("quit");
	}
      return;
    }
  if((sCommand=="PokeTracker"))
    {
      mbUserPressedSpacebar = true;
      return;
    }
    
  
  cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; */

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
void Tracker::TrackForInitialMap()
{
  // MiniPatch tracking threshhold.
  static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;
  

  initscr(); //initialize the ncurse mode
  //newterm(getenv("TERM"), stdout, stdin);
  //sleep(3000);
  cbreak();  //has something to do with input key press

  if(getch()==' ') //looking fr spacebar
  {
	  mbUserPressedSpacebar= true;
  }

  endwin();
  // What stage of initial tracking are we at?
  if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED) 
    {
      if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
	{
	  mbUserPressedSpacebar = false;
	  TrailTracking_Start();
	  mnInitialStage = TRAIL_TRACKING_STARTED;
	}
      else
	mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map." << endl;
      return;
    };
  
  if(mnInitialStage == TRAIL_TRACKING_STARTED)
  {
	  int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
	  if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
	  {
		  Reset();
		  return;
	  }

	  // If the user pressed spacebar here, use trails to run stereo and make the intial map..
      if(mbUserPressedSpacebar)
      {
    	  mbUserPressedSpacebar = false;
    	  vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs

    	  //cout<< "Loop teeation at line 422 tracker.cc mlTrails.begin(); i!=mlTrails.end()"<<std::distance(mlTrails.begin(),mlTrails.end())<<endl;

    	  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
    		  vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos,
    				  i->irCurrentPos));
    	  mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
    	  mnInitialStage = TRAIL_TRACKING_COMPLETE;
      }
      else
    	  mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init." << endl;
    }
}

// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start()
{
	mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
	mFirstKF = mCurrentKF;
	vector<pair<double,ImageRef> > vCornersAndSTScores;

	//cout<< "Loop teeation at line 442 tracker.cc mCurrentKF.aLevels[0].vCandidates.size()"<<mCurrentKF.aLevels[0].vCandidates.size()<<endl;

	for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++)  // Copy candidates into a trivially sortable vector
	{                                                                     // so that we can choose the image corners with max ST score
		Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
		if(!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
			continue;
		vCornersAndSTScores.push_back(pair<double,ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
	};
	sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
	int nToAdd = GV2.GetInt("MaxInitialTrails", 1000, SILENT);



	//cout<< "Loop teeation at line 454 tracker.cc vCornersAndSTScores.size()"<<vCornersAndSTScores.size()<<endl;
	for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
	{
		if(!mCurrentKF.aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
			continue;
		Trail t;
		t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
		t.irInitialPos = vCornersAndSTScores[i].second;
		t.irCurrentPos = t.irInitialPos;
		mlTrails.push_back(t);
		nToAdd--;
	}
	mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.
}

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int Tracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;
  if(mbDraw)
    {
      glPointSize(5);
      glLineWidth(2);
      glEnable(GL_POINT_SMOOTH);
      glEnable(GL_LINE_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glBegin(GL_LINES);
    }
  
  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = mCurrentKF.aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];
  
//cout<<"calling trailTracking_advance agaion......."<<endl;
  //cout<< "Loop teeation at line 490 tracker.cc mlTrails.begin(),mlTrails.end()"<<std::distance(mlTrails.begin(),mlTrails.end())<<endl;


 /* -----------------original codes------------------------------
  double i491=get_wall_time();
  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
	  list<Trail>::iterator next = i; next++;

      Trail &trail = *i;
      ImageRef irStart = trail.irCurrentPos;
      ImageRef irEnd = irStart;
      bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
      if(bFound)
	{
	  // Also find backwards in a married-matches check
	  BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
	  ImageRef irBackWardsFound = irEnd;
	  bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
	  if((irBackWardsFound - irStart).mag_squared() > 2)
	    bFound = false;
	  
	  trail.irCurrentPos = irEnd;
	  nGoodTrails++;
	}
      if(mbDraw)
	{
	  if(!bFound)
	    glColor3f(0,1,1); // Failed trails flash purple before dying.
	  else
	   glColor3f(1,1,0);
	  glVertex(trail.irInitialPos);
	  if(bFound) glColor3f(1,0,0);
	  glVertex(trail.irCurrentPos);
	}
      if(!bFound) // Erase from list of trails if not found this frame.
	{
	  mlTrails.erase(i);
	}
	  i = next;
    }

  double i529=get_wall_time();
 // cout<<"Tracker prospective opencl candidate: "<<(i529-i491)<<endl;
-------------------------------------------------------------------------------*/


  int loopSize=std::distance(mlTrails.begin(),mlTrails.end());

    clTrail* Ktrail;
    Ktrail= (clTrail*)malloc(loopSize*sizeof(clTrail));



  clImage* currentImage;
  currentImage=(clImage*)malloc(sizeof(clImage));
    currentImage->image_size_x=lCurrentFrame.im.size().x;
    currentImage->image_size_y=lCurrentFrame.im.size().y;
    currentImage->image_stride=lCurrentFrame.im.row_stride();
    for(int im=0;im<currentImage->image_size_x*currentImage->image_size_y;im++)
    {

    	currentImage->im[im]=	lCurrentFrame.im.data()[im];
    }


    clImage* prevImage;
    prevImage=(clImage*)malloc(sizeof(clImage));
    prevImage->image_size_x=lPreviousFrame.im.size().x;
    prevImage->image_size_y=lPreviousFrame.im.size().y;
    prevImage->image_stride=lPreviousFrame.im.row_stride();
    for(int im=0;im<prevImage->image_size_x*prevImage->image_size_y;im++)
    {

    	prevImage->im[im]=lPreviousFrame.im.data()[im];
    }

    clTemplate* mimOrigPatch;
    mimOrigPatch=(clTemplate*)malloc(loopSize*sizeof(clTemplate));

    int* KnGoodTrails=(int*)malloc(loopSize*sizeof(int));

    clTemplate* mimOrigPatchFmBackwrds;
    mimOrigPatchFmBackwrds=(clTemplate*)malloc(loopSize*sizeof(clTemplate));

    CVD::ImageRef imRefForBck( 2 * BackwardsPatch.mnHalfPatchSize + 1 , 2 * BackwardsPatch.mnHalfPatchSize + 1);
    BackwardsPatch.mimOrigPatch.resize(imRefForBck);
    int iterator=0;

    for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
    	list<Trail>::iterator next = i; next++;

    	Trail &trail = *i;

    	Ktrail[iterator].irCurrentPos[0]=trail.irCurrentPos.x;
    	Ktrail[iterator].irCurrentPos[1]=trail.irCurrentPos.y;
    	Ktrail[iterator].irInitialPos[0]=trail.irInitialPos.x;
    	Ktrail[iterator].irInitialPos[1]=trail.irInitialPos.y;
    	Ktrail[iterator].bFound=0;
    	KnGoodTrails[iterator]=0;

    	mimOrigPatch[iterator].image_size_x=trail.mPatch.mimOrigPatch.size().x;
    	mimOrigPatch[iterator].image_size_y=trail.mPatch.mimOrigPatch.size().y;
    	mimOrigPatch[iterator].image_stride=trail.mPatch.mimOrigPatch.row_stride();

    	mimOrigPatchFmBackwrds[iterator].image_size_x=BackwardsPatch.mimOrigPatch.size().x;
    	mimOrigPatchFmBackwrds[iterator].image_size_y=BackwardsPatch.mimOrigPatch.size().y;
    	mimOrigPatchFmBackwrds[iterator].image_stride=BackwardsPatch.mimOrigPatch.row_stride();

    	 assert(trail.mPatch.mimOrigPatch.size().x==9);
    	 assert(trail.mPatch.mimOrigPatch.size().y==9);

      	 assert(BackwardsPatch.mimOrigPatch.size().x==9);
        	 assert(BackwardsPatch.mimOrigPatch.size().y==9);

    	for(int im=0;im<mimOrigPatch[iterator].image_size_x*mimOrigPatch[iterator].image_size_y;im++)
    	{

    		mimOrigPatch[iterator].im[im]=trail.mPatch.mimOrigPatch.data()[im] ;
    	}

    	for(int im=0;im<mimOrigPatchFmBackwrds[iterator].image_size_x*mimOrigPatchFmBackwrds[iterator].image_size_y;im++)
    	{

    		mimOrigPatchFmBackwrds[iterator].im[im]=BackwardsPatch.mimOrigPatch.data()[im] ;
    	}

    	iterator++;
    	i = next;
    }


    int vCurrentCornerSize=lCurrentFrame.vCorners.size();
    int vPrevCornerSize=lPreviousFrame.vCorners.size();
    int kmnMaxSSD=BackwardsPatch.mnMaxSSD;
    int kmnHalfPatchSize=BackwardsPatch.mnHalfPatchSize;

    	clvCorners* vCurntCornerx;
       vCurntCornerx= (clvCorners*)malloc(vCurrentCornerSize*sizeof(clvCorners));

       clvCorners* vCornerxPrev;
       vCornerxPrev= (clvCorners*)malloc(vPrevCornerSize*sizeof(clvCorners));

       for(int k=0;k<vCurrentCornerSize;k++)
       {
    	   vCurntCornerx[k].vCon[0]=lCurrentFrame.vCorners[k][0];
    	   vCurntCornerx[k].vCon[1]=lCurrentFrame.vCorners[k][1];
       }
       for(int k=0;k<vPrevCornerSize;k++)
       {
    	   vCornerxPrev[k].vCon[0]=lPreviousFrame.vCorners[k][0];
    	   vCornerxPrev[k].vCon[1]=lPreviousFrame.vCorners[k][1];
       }

       char* filename="TrailPatch.cl";
       char* kernelName="TrailPatch";


 /*      char* filename="TestKernel.cl";
       char* kernelName="VectorAdd";
*/
       //start testing the CPU values for image data:
/*
    	printf("CPU data from Tracker.cc file\n");

       	printf("CPU mimOrigPatchFmBackwrds im[0] :%d\n",mimOrigPatchFmBackwrds[4].im[112]);

       	printf("CPU PreviousImg[gid].[1] :%d\n",prevImage->im[355]);
       	printf("CPU mimOrigPatch[gid].[1] :%d\n",mimOrigPatch[4].im[9]);
       	printf("CPU Imageg[gid] :%d\n",currentImage->im[428]);

*/

       //end


    /*   if(loopSize>0)
       {

    	   //if(globalCounterForKernelCall==0)
    	   //{

    		   //TestCallKernel(kernelName,filename);
    	   TrailPatch_1(kernelName,filename,loopSize,
    			   Ktrail,
    			   KnGoodTrails,
    			   currentImage,
    			   prevImage,
    			   mimOrigPatch,
    			   mimOrigPatchFmBackwrds,
    			   vCurntCornerx,
    			   vCornerxPrev,
    			   kmnMaxSSD,
    			   vCurrentCornerSize,
    			   vPrevCornerSize,
    			   kmnHalfPatchSize);

    	   //globalCounterForKernelCall++;
    	  // }


       }



      // cout<<"variahle values: "<<globalCounterForKernelCall<<endl;
     */  int kntrails=0;
       bool* bFoundf=(bool*)malloc(std::distance(mlTrails.begin(),mlTrails.end())*sizeof(bool));
       int countf=0;
  /*    // FILE *fpi12 = fopen("kernel_data.txt", "w");
     for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
    	list<Trail>::iterator next = i; next++;

    	Trail &trail = *i;
    	trail.irCurrentPos.x=Ktrail[countf].irCurrentPos[0];
    	trail.irCurrentPos.y=Ktrail[countf].irCurrentPos[1];

    	bFoundf[countf]=Ktrail[countf].bFound;//*
    	kntrails=kntrails+KnGoodTrails[countf];
    	countf++;
    	i = next;
    }
  //  cout<<"nGoodTrails kernerl: "<<kntrails<<endl;
   // fclose(fpi12);
    nGoodTrails=kntrails;
*/

       // ----------------------------modified one---------------------


  	double i688=get_wall_time();
       for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
       {
    	   list<Trail>::iterator next = i; next++;

    	   Trail &trail = *i;
    	   ImageRef irStart = trail.irCurrentPos;
    	   ImageRef irEnd = irStart;


    	   bFoundf[countf] = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners,countf);


    	   /*if(countf==4)
		  cout<<"CPU: "<<irStart.x<<endl;*/
    	   if(bFoundf[countf])
    	   {
    		   // Also find backwards in a married-matches check
    		   BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);

    		   unsigned char *imagf=lCurrentFrame.im.data();
    		   int str=lCurrentFrame.im.row_stride();

    		   ImageRef irBackWardsFound = irEnd;
    		   bFoundf[countf] = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);


    		   if((irBackWardsFound - irStart).mag_squared() > 2)
    			   bFoundf[countf] = false;

    		   trail.irCurrentPos = irEnd;
    		   nGoodTrails++;

    	   }

    	   countf++;
    	   i = next;
       }

      // cout<<"nGoodTrails CPU: "<<nGoodTrails<<endl;
    	double i738=get_wall_time();
    	cout<<"cpu execution time.........."<<(i738-i688)<<endl;

    	int countfz=0;
    	for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    	{
    		list<Trail>::iterator next = i; next++;
    		Trail &trail = *i;
    		if(mbDraw)
    		{
    			if(! bFoundf[countfz])
    				glColor3f(0,1,1); // Failed trails flash purple before dying.
    			else
    				glColor3f(1,1,0);
    			glVertex(trail.irInitialPos);
    			if(bFoundf[countfz]) glColor3f(1,0,0);
    			glVertex(trail.irCurrentPos);
    		}
    		if(!bFoundf[countfz]) // Erase from list of trails if not found this frame.
    		{
    			mlTrails.erase(i);
    		}
    		countfz++;
    		i = next;
    	}

    	if(mbDraw)
    		glEnd();

    	mPreviousFrameKF = mCurrentKF;
    	return nGoodTrails;
}


/*
int Tracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;
  if(mbDraw)
    {
      glPointSize(5);
      glLineWidth(2);
      glEnable(GL_POINT_SMOOTH);
      glEnable(GL_LINE_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glBegin(GL_LINES);
    }

  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = mCurrentKF.aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

 /* int loopSize=std::distance(mlTrails.begin(),mlTrails.end());

  clTrail* Ktrail;
  Ktrail= (clTrail*)malloc(loopSize*sizeof(clTrail));

  vCorners* vCornx;
  vCornx= (vCorners*)malloc(loopSize*sizeof(vCorners));

  clImage* currentImage;
  currentImage->image_size_x=lCurrentFrame.im.size().x;
  currentImage->image_size_y=lCurrentFrame.im.size().y;
  currentImage->image_stride=lCurrentFrame.im.row_stride();
  for(int im=0;im<currentImage->image_size_x*currentImage->image_size_y;im++)
  {

	  currentImage->im[im]=	lCurrentFrame.im.data()[im];
  }


  clImage* prevImage;
  prevImage->image_size_x=lPreviousFrame.im.size().x;
  prevImage->image_size_y=lPreviousFrame.im.size().y;
  prevImage->image_stride=lPreviousFrame.im.row_stride();
  for(int im=0;im<prevImage->image_size_x*prevImage->image_size_y;im++)
  {

	  prevImage->im[im]=	lPreviousFrame.im.data()[im];
  }

*/
/*
  bool* bFoundf=(bool*)malloc(std::distance(mlTrails.begin(),mlTrails.end())*sizeof(bool));
  int countf=0;
  /*----------------------------modified one---------------------*/
 /* for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
  {
	  list<Trail>::iterator next = i; next++;

	  Trail &trail = *i;
	  ImageRef irStart = trail.irCurrentPos;
	  ImageRef irEnd = irStart;

	  bFoundf[countf] = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);

	  if(bFoundf[countf])
	  {
		  // Also find backwards in a married-matches check
		  BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
		  cout<<"image size x: "<<lCurrentFrame.im.size().x;
		  cout<<"image size y: "<<lCurrentFrame.im.size().y;
		  unsigned char *imagf=lCurrentFrame.im.data();
		  int str=lCurrentFrame.im.row_stride();

  		  ImageRef irBackWardsFound = irEnd;
		  bFoundf[countf] = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
		  if((irBackWardsFound - irStart).mag_squared() > 2)
			  bFoundf[countf] = false;

		  trail.irCurrentPos = irEnd;
		  nGoodTrails++;
	  }
	  countf++;
	  i = next;
  }

  int countfz=0;
  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
  {
	  list<Trail>::iterator next = i; next++;
	  Trail &trail = *i;
	  if(mbDraw)
	  {
		  if(! bFoundf[countfz])
			  glColor3f(0,1,1); // Failed trails flash purple before dying.
		  else
			  glColor3f(1,1,0);
		  glVertex(trail.irInitialPos);
		  if(bFoundf[countfz]) glColor3f(1,0,0);
		  glVertex(trail.irCurrentPos);
	  }
	  if(!bFoundf[countfz]) // Erase from list of trails if not found this frame.
	  {
		  mlTrails.erase(i);
	  }
	  countfz++;
	  i = next;
  }

  if(mbDraw)
    glEnd();

  mPreviousFrameKF = mCurrentKF;
  return nGoodTrails;
}*/

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap()
{
	// Some accounting which will be used for tracking quality assessment:
	for(int i=0; i<LEVELS; i++)
		manMeasAttempted[i] = manMeasFound[i] = 0;

	// The Potentially-Visible-Set (PVS) is split into pyramid levels.
	vector<TrackerData*> avPVS[LEVELS];
	for(int i=0; i<LEVELS; i++)
		avPVS[i].reserve(500); //mnl>>This function attempts to reserve enough memory for the
	//*  %vector to hold the specified number of elements

	double i537=get_wall_time();
	// For all points in the map..


	 //cout<< "Loop teeation at line 561 tracker.cc mMap.vpPoints.size()"<<mMap.vpPoints.size()<<endl;

	for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
		MapPoint &p= *(mMap.vpPoints[i]);
		// Ensure that this map point has an associated TrackerData struct.
		if(!p.pTData) p.pTData = new TrackerData(&p);
		TrackerData &TData = *p.pTData;

		// Project according to current view, and if it's not in the image, skip.
		TData.Project(mse3CamFromWorld, mCamera);
		if(!TData.bInImage)
			continue;

		// Calculate camera projection derivatives of this point.
		TData.GetDerivsUnsafe(mCamera);

		// And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
		TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
		if(TData.nSearchLevel == -1)
			continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

		// Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
		TData.bSearched = false;
		TData.bFound = false;
		avPVS[TData.nSearchLevel].push_back(&TData);
	};

	double i566=get_wall_time();

	//cout<<"TIme for loop1 at between 537-566 "<<endl;//mnl>>print the time
	//cout<<(i566-i537)<<endl;

	double i571=get_wall_time();
	// Next: A large degree of faffing about and deciding which points are going to be measured!
	// First, randomly shuffle the individual levels of the PVS.
	for(int i=0; i<LEVELS; i++)
		random_shuffle(avPVS[i].begin(), avPVS[i].end());

	double i576=get_wall_time();
	//cout<<"TIme for loop1 at between 576-571 "<<endl;//mnl>>print the time
	// cout<<(i576-i571)<<endl;
	// The next two data structs contain the list of points which will next
	// be searched for in the image, and then used in pose update.
	vector<TrackerData*> vNextToSearch;
	vector<TrackerData*> vIterationSet;

	// Tunable parameters to do with the coarse tracking stage:
	static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
	static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
	static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
	static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
	static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
	static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.

	unsigned int nCoarseMax = *gvnCoarseMax;
	unsigned int nCoarseRange = *gvnCoarseRange;

	mbDidCoarse = false;

	// Set of heuristics to check if we should do a coarse tracking stage.
	bool bTryCoarse = true;
	if(*gvnCoarseDisabled ||
			mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
			nCoarseMax == 0)
		bTryCoarse = false;
	if(mbJustRecoveredSoUseCoarse)
	{
		bTryCoarse = true;
		nCoarseMax *=2;
		nCoarseRange *=2;
		mbJustRecoveredSoUseCoarse = false;
	};

	// If we do want to do a coarse stage, also check that there's enough high-level
	// PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
	// with preference to LEVELS-1.
	if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin )
	{
		// Now, fill the vNextToSearch struct with an appropriate number of
		// TrackerDatas corresponding to coarse map points! This depends on how many
		// there are in different pyramid levels compared to CoarseMin and CoarseMax.

		if(avPVS[LEVELS-1].size() <= nCoarseMax)
		{ // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
			vNextToSearch = avPVS[LEVELS-1];
			avPVS[LEVELS-1].clear();
		}
		else
		{ // ..otherwise choose nCoarseMax at random, again removing from the PVS list.

			//cout<< "Loop teeation at line 651 tracker.cc nCoarseMax"<<nCoarseMax<<endl;
			for(unsigned int i=0; i<nCoarseMax; i++)
				vNextToSearch.push_back(avPVS[LEVELS-1][i]);
			avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
		}

		// If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
		if(vNextToSearch.size() < nCoarseMax)
		{
			unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
			//cout<< "Loop teeation at line 667 tracker.cc nMoreCoarseNeeded"<<nMoreCoarseNeeded<<endl;
			if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
			{
				vNextToSearch = avPVS[LEVELS-2];
				avPVS[LEVELS-2].clear();
			}
			else
			{
				for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
					vNextToSearch.push_back(avPVS[LEVELS-2][i]);
				avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
			}
		}
		double i649=get_wall_time();
		// Now go and attempt to find these points in the image!
		unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
		vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
		if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
		{
			mbDidCoarse = true;
			// cout<< "Loop teeation at line 685 tracker.cc vIterationSet.size()"<<vIterationSet.size()<<endl;

			for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
			{
				if(iter != 0)
				{ // Re-project the points on all but the first iteration.
					for(unsigned int i=0; i<vIterationSet.size(); i++)
						if(vIterationSet[i]->bFound)
							vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
				}
				for(unsigned int i=0; i<vIterationSet.size(); i++)
					if(vIterationSet[i]->bFound)
						vIterationSet[i]->CalcJacobian();
				double dOverrideSigma = 0.0;
				// Hack: force the MEstimator to be pretty brutal
				// with outliers beyond the fifth iteration.
				if(iter > 5)
					dOverrideSigma = 1.0;

				// Calculate and apply the pose update...
				Vector<6> v6Update =
						CalcPoseUpdate(vIterationSet, dOverrideSigma);
				mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
			};
		}

		double i680=get_wall_time();
		//cout<<"TIme for loop1 at between 680-649 "<<endl;//mnl>>print the time
		//cout<<(i680-i649)<<endl;
	};

	// So, at this stage, we may or may not have done a coarse tracking stage.
	// Now do the fine tracking stage. This needs many more points!
	double i687=get_wall_time();
	int nFineRange = 10;  // Pixel search range for the fine stage.
	if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
		nFineRange = 5;

	// What patches shall we use this time? The high-level ones are quite important,
	// so do all of these, with sub-pixel refinement.
	{
		int l = LEVELS - 1;
		// cout<< "Loop teeation at line 723 tracker.cc avPVS[l].size()"<<avPVS[l].size()<<endl;

		for(unsigned int i=0; i<avPVS[l].size(); i++)
			avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
		SearchForPoints(avPVS[l], nFineRange, 8);
		for(unsigned int i=0; i<avPVS[l].size(); i++)
			vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
	};

	double i703=get_wall_time();
	// cout<<"TIme for loop1 at between i703-i687 "<<endl;//mnl>>print the time
	//cout<<(i703-i687)<<endl;

	double i707=get_wall_time();
	// All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
	vNextToSearch.clear();
	for(int l=LEVELS - 2; l>=0; l--)
		for(unsigned int i=0; i<avPVS[l].size(); i++)
			vNextToSearch.push_back(avPVS[l][i]);

	double i714=get_wall_time();
	// cout<<"TIme for loop1 at between i714-i707 "<<endl;//mnl>>print the time
	//cout<<(i714-i707)<<endl;


	double i719=get_wall_time();
	// But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
	// ourselves to 1000, and choose these randomly.
	static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
	int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
	if(nFinePatchesToUse < 0)
		nFinePatchesToUse = 0;
	if((int) vNextToSearch.size() > nFinePatchesToUse)
	{
		random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
		vNextToSearch.resize(nFinePatchesToUse); // Chop!
	};

	double i732=get_wall_time();
	// If we did a coarse tracking stage: re-project and find derivs of fine points

	// cout<< "Loop teeation at line 764 tracker.cc vNextToSearch.size()"<<vNextToSearch.size()<<endl;
	if(mbDidCoarse)
		for(unsigned int i=0; i<vNextToSearch.size(); i++)
			vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);

	double i737=get_wall_time();
	// cout<<"TIme for loop1 at between i737-i732 "<<endl;//mnl>>print the time
	// cout<<(i737-i732)<<endl;

	double i742=get_wall_time();

	// Find fine points in image:
	//cout<<"calling time consuming SearchforPoints functions...................: "<<endl;

	//ConvertDatatoDeviceType(vNextToSearch, nFineRange, 0); //mnl>>calling kernel; transferring data to device

	double i743=get_wall_time();
	SearchForPointsCustom(vNextToSearch, nFineRange, 0);
	double i746=get_wall_time();
	// cout<<"TIme for most time consuming SearchForPoints i746-i743............................... "<<endl;//mnl>>print the time
	 //cout<<(i746-i743)<<endl;
	// And attach them all to the end of the optimisation-set.
	// cout<< "Loop teeation at line 786 tracker.cc vNextToSearch.size()"<<vNextToSearch.size()<<endl;

	for(unsigned int i=0; i<vNextToSearch.size(); i++)
		vIterationSet.push_back(vNextToSearch[i]);


	double i749=get_wall_time();
	//cout<<"TIme for loop1 at between i749-i742 "<<endl;//mnl>>print the time
	//cout<<(i749-i742)<<endl;

	double i754=get_wall_time();
	// Again, ten gauss-newton pose update iterations.
	Vector<6> v6LastUpdate;
	v6LastUpdate = Zeros;

	// cout<< "Loop teeation at line 814 tracker.cc vIterationSet.size()"<<vIterationSet.size()<<endl;

	for(int iter = 0; iter<10; iter++)
	{
		bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
		// reprojection at every iteration - it really isn't necessary!
		if(iter == 0 || iter == 4 || iter == 9)
			bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
		else                            // iterations is for M-Estimator convergence rather than
			bNonLinearIteration = false;  // linearisation effects.

		if(iter != 0)   // Either way: first iteration doesn't need projection update.
		{
			if(bNonLinearIteration)
			{
				for(unsigned int i=0; i<vIterationSet.size(); i++)
					if(vIterationSet[i]->bFound)
						vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
			}
			else
			{
				for(unsigned int i=0; i<vIterationSet.size(); i++)
					if(vIterationSet[i]->bFound)
						vIterationSet[i]->LinearUpdate(v6LastUpdate);
			};
		}

		if(bNonLinearIteration)
			for(unsigned int i=0; i<vIterationSet.size(); i++)
				if(vIterationSet[i]->bFound)
					vIterationSet[i]->CalcJacobian();

		// Again, an M-Estimator hack beyond the fifth iteration.
		double dOverrideSigma = 0.0;
		if(iter > 5)
			dOverrideSigma = 16.0;

		// Calculate and update pose; also store update vector for linear iteration updates.
		Vector<6> v6Update =
				CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
		mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
		v6LastUpdate = v6Update;
	};
	double i798=get_wall_time();
	//cout<<"TIme for loop1 at between i798-i754 "<<endl;//mnl>>print the time
	//cout<<(i798-i754)<<endl;

	double i785=get_wall_time();
	// cout<<"TIme for loop1 at between i785-i719 "<<endl;//mnl>>print the time
	// cout<<(i785-i719)<<endl;


	double i792=get_wall_time();
	if(mbDraw)
	{
		glPointSize(6);
		glEnable(GL_BLEND);
		glEnable(GL_POINT_SMOOTH);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBegin(GL_POINTS);
		for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
				it!= vIterationSet.rend();
				it++)
		{
			if(! (*it)->bFound)
				continue;
			glColor(gavLevelColors[(*it)->nSearchLevel]);
			glVertex((*it)->v2Image);
		}
		glEnd();
		glDisable(GL_BLEND);
	}

	// Update the current keyframe with info on what was found in the frame.
	// Strictly speaking this is unnecessary to do every frame, it'll only be
	// needed if the KF gets added to MapMaker. Do it anyway.
	// Export pose to current keyframe:
	mCurrentKF.se3CfromW = mse3CamFromWorld;

	// Record successful measurements. Use the KeyFrame-Measurement struct for this.

	 //cout<< "Loop teeation at line 883 tracker.cc distance(vIterationSet.begin(),vIterationSet.end())"<<std::distance(vIterationSet.begin(),vIterationSet.end())<<endl;

	mCurrentKF.mMeasurements.clear();
	for(vector<TrackerData*>::iterator it = vIterationSet.begin();
			it!= vIterationSet.end();
			it++)
	{
		if(! (*it)->bFound)
			continue;
		Measurement m;
		m.v2RootPos = (*it)->v2Found;
		m.nLevel = (*it)->nSearchLevel;
		m.bSubPix = (*it)->bDidSubPix;
		mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
	}

	// Finally, find the mean scene depth from tracked features
	{
		double dSum = 0;
		double dSumSq = 0;
		int nNum = 0;
		for(vector<TrackerData*>::iterator it = vIterationSet.begin();
				it!= vIterationSet.end();
				it++)
			if((*it)->bFound)
			{
				double z = (*it)->v3Cam[2];
				dSum+= z;
				dSumSq+= z*z;
				nNum++;
			};
		if(nNum > 20)
		{
			mCurrentKF.dSceneDepthMean = dSum/nNum;
			mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
		}
	}

	double i854=get_wall_time();
	//cout<<"TIme for loop1 at between i854-i792 "<<endl;//mnl>>print the time
	//cout<<(i854-i792)<<endl;
}

// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
 // cout<<endl<<" line 935 vtd. size is............................................. :";
 // cout<<vTD.size()<<endl; //mnl<< size is 27 and 973
  double MakeTemplateCoarseContTIme=0.0;
  double FindPatchCoarseTime=0.0;
  double IterateSubPixToConvergenceTime=0.0;
  double GetCoarsePosAsVectorTIme=0.0;
  double MakeSubPixTemplateTIme=0.0;

  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
      {
	  TrackerData &TD = *vTD[i];
	       PatchFinder &Finder = TD.Finder;
	 Finder.MakeTemplateCoarseCont(TD.Point);

      }
 // cout<<"calling from search for points....."<<endl;
  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
    {

	  double i895=get_wall_time();

      TrackerData &TD = *vTD[i];
      PatchFinder &Finder = TD.Finder;

      double i900=get_wall_time();
   //  Finder.MakeTemplateCoarseCont(TD.Point);

      double i903=get_wall_time();
     // cout<<"Time for MakeTemplateCoarseCont "<<endl;//mnl>>print the time
     // cout<<(i903-i900)<<endl;
      MakeTemplateCoarseContTIme=MakeTemplateCoarseContTIme+(i903-i900);

      if(Finder.TemplateBad())
      {
    	  TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
    	  continue;
      }
      manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta

      double i914=get_wall_time();

      bool bFound;

    	  bFound = Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange,i);

      double i919=get_wall_time();
      //cout<<"Time for FindPatchCoarse "<<endl;//mnl>>print the time
      //cout<<(i919-i914)<<endl;
      FindPatchCoarseTime=FindPatchCoarseTime+(i919-i914);
     // cout<<"=================================";

      TD.bSearched = true;

      if(!bFound) 
      {
    	  TD.bFound = false;
    	  continue;
      }
      
      TD.bFound = true;
      TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
      int nfoundpre;
      if (i==4)  nfoundpre=nFound;
      nFound++;

      manMeasFound[Finder.GetLevel()]++;



      // Found the patch in coarse search - are Sub-pixel iterations wanted too?//mnl not called most of the time. commentng
      if(nSubPixIts > 0)
      {
    	  TD.bDidSubPix = true;

    	  double i974=get_wall_time();
    	  Finder.MakeSubPixTemplate();
    	  double i976=get_wall_time();
    	//  cout<<"Time for MakeSubPixTemplate "<<endl;//mnl>>print the time
    	     //       cout<<(i976-i974)<<endl;
    	            MakeSubPixTemplateTIme=MakeSubPixTemplateTIme+(i976-i974);


    	  double i945=get_wall_time();

    	  bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);

          double i949=get_wall_time();
         // cout<<"Time for IterateSubPixToConvergence "<<endl;//mnl>>print the time
         // cout<<(i949-i945)<<endl;
          IterateSubPixToConvergenceTime=IterateSubPixToConvergenceTime+(i949-i945);


    	  if(!bSubPixConverges)
    	  { // If subpix doesn't converge, the patch location is probably very dubious!
    		  TD.bFound = false;
    		  nFound--; //mnl>>nFound is changed here...//
    		  manMeasFound[Finder.GetLevel()]--;
    		  continue;
    	  }
    	  TD.v2Found = Finder.GetSubPixPos();
      }
      else

      {
    	  double i964=get_wall_time();

    	  TD.v2Found = Finder.GetCoarsePosAsVector();

    	  double i968=get_wall_time();
    	 // cout<<"Time for GetCoarsePosAsVector "<<endl;//mnl>>print the time
    	 // cout<<(i968-i964)<<endl;
    	  GetCoarsePosAsVectorTIme=GetCoarsePosAsVectorTIme+(i968-i964);
    	  TD.bDidSubPix = false;
      }

      //double i944=get_wall_time();
       //cout<<"TIme single iteration inside loop of search for points: "<<endl;//mnl>>print the time
       //cout<<(i944-i895)<<endl;
    }

  return nFound;
};


int Tracker::SearchForPointsCustom(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  //cout<<endl<<"vtd. size is............................................. :";
 // cout<<vTD.size()<<endl; //mnl<< size is 27 and 973
  double MakeTemplateCoarseContTIme=0.0;
  double FindPatchCoarseTime=0.0;
  double IterateSubPixToConvergenceTime=0.0;
  double GetCoarsePosAsVectorTIme=0.0;
  double MakeSubPixTemplateTIme=0.0;

  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
      {
	  TrackerData &TD = *vTD[i];
	       PatchFinder &Finder = TD.Finder;
	 Finder.MakeTemplateCoarseCont(TD.Point);

      }
 // cout<<"calling from search for points....."<<endl;
  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
    {
      // First, attempt a search at pixel locations which are FAST corners.
      // (PatchFinder::FindPatchCoarse)
	//  cout<<"iteration number.....................:";
	 // cout<<i<<endl;

	  double i895=get_wall_time();

      TrackerData &TD = *vTD[i];
      PatchFinder &Finder = TD.Finder;

      //testing values of td
     /* if(i==0)
      {
    	  cout<<"inside search for points:....."<<endl;
    	  cout<<"V2Image[0] value:..."<<endl;
    	  cout<<TD.v2Image[0]<<endl;
    	  cout<<"V2Image[1] value:..."<<endl;
    	  cout<<TD.v2Image[1]<<endl;
      }*/

     /* if(i==0)
      {
    	  cout<<"inside search for points:....."<<endl;
    	  cout<<"v2Found[0] value:..."<<endl;
    	  cout<<TD.v2Found[0]<<endl;
    	  cout<<"v2Found[1] value:..."<<endl;
    	  cout<<TD.v2Found[1]<<endl;
      }*/

      double i900=get_wall_time();
   //  Finder.MakeTemplateCoarseCont(TD.Point);

      double i903=get_wall_time();
     // cout<<"Time for MakeTemplateCoarseCont "<<endl;//mnl>>print the time
     // cout<<(i903-i900)<<endl;
      MakeTemplateCoarseContTIme=MakeTemplateCoarseContTIme+(i903-i900);

      if(Finder.TemplateBad())
      {
    	  TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
    	  continue;
      }
      manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta

      double i914=get_wall_time();

      bool bFound;

    	  bFound = Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange,i);

      double i919=get_wall_time();

      FindPatchCoarseTime=FindPatchCoarseTime+(i919-i914);


      TD.bSearched = true;

      if(!bFound)
      {
    	  TD.bFound = false;
    	  continue;
      }

      TD.bFound = true;
      TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());

      nFound++;

      manMeasFound[Finder.GetLevel()]++;


    	  double i964=get_wall_time();

    	  TD.v2Found = Finder.GetCoarsePosAsVector();

    	  double i968=get_wall_time();
    	 // cout<<"Time for GetCoarsePosAsVector "<<endl;//mnl>>print the time
    	 // cout<<(i968-i964)<<endl;
    	  GetCoarsePosAsVectorTIme=GetCoarsePosAsVectorTIme+(i968-i964);
    	  TD.bDidSubPix = false;


    }


  return nFound;
};




//------------------------------------------------------------------
//mnl>>ConvertDatatoDeviceType function is custom function to transfer data from the
//custom data type of host to device type so that kernel can work on the data
//----------------------------------------------------------------------


void Tracker::ConvertDatatoDeviceType(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
	/*double** v2Image; //this was a vector type in tracker data
	bool* bInImage;
	bool* bPotentiallyVisible;
	bool* bSearched;
	bool* bFound;
	bool* bDidSubPix;
	double** v2Found;    //this was a vector type in tracker data
	double* dSqrtInvNoise;
*/
	for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
	{
		TrackerData &TD = *vTD[i];
		PatchFinder &Finder = TD.Finder;
		Finder.MakeTemplateCoarseCont(TD.Point);

	}

	//--------allocating memory-----------//
	int* nFound=(int*)malloc(vTD.size() * sizeof(int));
	for( int i=0; i<vTD.size(); i++)
		{

			nFound[i]=0;
		}
#if(1)//allocate mempry
	cout<<"hello"<<endl;

	cl_int* vCornersEndIndex=(cl_int*)malloc(vTD.size()*sizeof(cl_int));

	clTrackerData* TD;
	TD= (clTrackerData*)malloc(vTD.size()*sizeof(clTrackerData));

	clLevel* Levelg;
	Levelg= (clLevel*)malloc(vTD.size()*sizeof(clLevel));

	clFinder* Finder;
	Finder=(clFinder*)malloc(vTD.size()*sizeof(clFinder));

	clImage* Imageg;
	Imageg=(clImage*)malloc(vTD.size()*sizeof(clImage));

	clImage* mimTemplate;
	mimTemplate=(clImage*)malloc(vTD.size()*sizeof(clImage));


	cl_uchar* image_data1;
	image_data1=(cl_uchar*)malloc(vTD.size()*sizeof(cl_uchar)); //mnl>>do not know how can I assign these values
	cl_uchar* image_data2;
	image_data2=(cl_uchar*)malloc(vTD.size()*sizeof(cl_uchar)); //mnl>>do not know how can I assign these values

	int vcorner_size=0;
	int vMaxcorner_size=0;
	int vCornerLut_size=0;



	cl_int* vCorners_starting_index;
	vCorners_starting_index=(cl_int*)malloc(vTD.size()*sizeof(cl_int));

	cl_int* vMaxCorners_starting_index;
	vMaxCorners_starting_index=(cl_int*)malloc(vTD.size()*sizeof(cl_int));
	cl_int* vCornerLut_starting_index;
	vCornerLut_starting_index=(cl_int*)malloc(vTD.size()*sizeof(cl_int));

	cl_int* vCorners_custom;
	vCorners_custom=(cl_int*)malloc(2*vTD.size()*sizeof(cl_int));

	cl_int* vCornerRowLUT_custom;
	vCornerRowLUT_custom=(cl_int*)malloc(vTD.size()*sizeof(cl_int));
	//cout<<"size of vlutL:"<<sizeof(vCornerRowLUT_custom);

	cl_int* vMaxCorners_custom;
	vMaxCorners_custom=(cl_int*)malloc(2*vTD.size()*sizeof(cl_int));


	cout<<"vtdvTD.size() in my functions......."<<vTD.size()<<endl;
	for( int i=0; i<vTD.size(); i++)
	{
		TrackerData &TDForCPU = *vTD[i];
		PatchFinder &Finder_CPU=TDForCPU.Finder;

		TD[i].v2Image[0]=vTD[i]->v2Image[0];
				//vTD[i]->v2Image[i][0]; //correctly assigned and tested;
		TD[i].v2Image[1]=vTD[i]->v2Image[1]; //correctly assigned and tested;
		TD[i].v2Found[0]=vTD[i]->v2Found[0];
		TD[i].v2Found[1]=vTD[i]->v2Found[1];

		TD[i].bInImage=vTD[i]->bInImage;
		TD[i].bPotentiallyVisible=vTD[i]->bPotentiallyVisible;
		TD[i].bSearched=vTD[i]->bSearched;
		TD[i].bFound=vTD[i]->bFound;
		TD[i].bDidSubPix=vTD[i]->bDidSubPix;
		TD[i].dSqrtInvNoise=vTD[i]->dSqrtInvNoise;

		Finder[i].mbTemplateBad=Finder_CPU.TemplateBad();
		Finder[i].mnSearchLevel=Finder_CPU.GetLevel();
		Finder[i].LevelScale=Finder_CPU.GetLevelScale();
		Finder[i].mirCenter[0]=Finder_CPU.GetmirCenter().x;
		Finder[i].mirCenter[1]=Finder_CPU.GetmirCenter().y;
		Finder[i].mnPatchSize=Finder_CPU.GetPatchSize();
		Finder[i].mnTemplateSum=Finder_CPU.GetTemplateSum();
		Finder[i].mnTemplateSumSq=Finder_CPU.GetTemplateSumQ();
		Finder[i].mnMaxSSD=Finder_CPU.mnMaxSSD;
		Finder[i].mirPredictedPos[0]=Finder_CPU.GetPredicatedPos().x;
		Finder[i].mirPredictedPos[1]=Finder_CPU.GetPredicatedPos().y;
		Finder[i].mbFound=Finder_CPU.GetmbFound();
		Finder[i].mv2CoarsePos[0]=Finder_CPU.Getmv2CoarsePos()[0];
		Finder[i].mv2CoarsePos[1]=Finder_CPU.Getmv2CoarsePos()[1];

		/*if(i==4)
		{
		cout<<"mnTemplateSum my code CPU"<<Finder[i].mnTemplateSum<<endl;
		cout<<"mnTemplateSum my code original CPU"<<Finder_CPU.GetTemplateSum()<<endl;
		}*/


		for(int im=0;im<4;im++)
		{
			Finder[i].manMeasAttempted[im]=manMeasAttempted[im];

		}

		for(int im=0;im<4;im++)
		{
			Finder[i].manMeasFound[im]=manMeasFound[im];

		}


		//getting the last index of vCorners for each level
		vCornersEndIndex[i]=std::distance(mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.begin(), mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.end())-1;

		for(int im=0;im<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT.size();im++)
		{
			Levelg[i].vCornerRowLUT[im]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT[im];

		}

		/*cout<<"max size"<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners.size()<<endl; //mnl>>size is zero
		cout<<"value"<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[4].x; //mnl>>then how this is returning a value??*/

		for(int im=0;im<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners.size();im++)
		{
			Levelg[i].vMaxCorners[im][0]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[im].x;
			Levelg[i].vMaxCorners[im][1]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[im].y;
			//cout<<"max org"<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[im].x<<endl;
			//cout<<"max cust"<<Levelg[i].vMaxCorners[im][1]<<endl;


		}
		/*if(i==4)
		{
			cout<<"sizevMax: "<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[100].x<<endl;
			cout<<"index3Max: "<<Levels[4].vMaxCorners[100][0]<<endl;


		}*/

		for(int im=0;im<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.size();im++)
		{
			Levelg[i].vCorners[im][0]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners[im].x;
			Levelg[i].vCorners[im][1]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners[im].y;
		}


		/*image_size_x[i]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.size().x;
		image_size_y[i]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.size().y;
		image_stride[i]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.row_stride();*/

		//cout<<"image index size: "<<image_size<<endl;
		//cout<<"loop index : "<<i<<endl;
		//cout<<"image ref size: "<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.size()<<endl;



		/*for(int im=0;im<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.size();im++)
		{
			vCorners_custom[vcorner_size+2*im]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners[im].x;
			vCorners_custom[vcorner_size+2*im+1]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners[im].y;

		}

		vCorners_starting_index[i]=vcorner_size;
		vcorner_size=vcorner_size+mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.size()*2;*/


		/*for(int im=0;im<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT.size();im++)
		{
			vCornerRowLUT_custom[vCornerLut_size+im]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT[im];
			//cout<<"originakl:::::::"<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT[im]<<endl;
			//cout<<"aaaadfrv"<<vCornerRowLUT_custom[vCornerLut_size+im]<<endl;

		}

		vCornerLut_starting_index[i]=vCornerLut_size;
		vCornerLut_size=vCornerLut_size+mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT.size();

		if(i==4)
		{
			cout<<"size: "<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT.size()<<endl;
			cout<<"index3: "<<vCornerLut_starting_index[i]<<endl;
			//cout<<"original value:"<<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT[110]<<endl;
			//cout<<"custom value:"<<vCornerRowLUT_custom[vCornerLut_starting_index[4]+110]<<endl;

		}*/
		/*for(int im=0;im<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners.size();im++)
		{
			vMaxCorners_custom[vMaxcorner_size+2*im]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[im].x;
			vMaxCorners_custom[vMaxcorner_size+2*im+1]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[im].y;
		}

		vMaxCorners_starting_index[i]=vMaxcorner_size;
		vMaxcorner_size=vMaxcorner_size+mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners.size()*2;*/

		//cout<<"size of image: "<<sizeof(mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.data())<<endl;
		//image_size=image_size+image_stride[i];

		Imageg[i].image_stride=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.row_stride();
		Imageg[i].image_size_y=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.size().y;
		Imageg[i].image_size_x=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.size().x;
		/*if(i==4)
		{
			cout<<"Image size y in my code CPU: "<<Imageg[i].image_size_y<<endl;
		}*/
		for(int im=0;im<Imageg[i].image_size_x*Imageg[i].image_size_y;im++)
		{
			Imageg[i].im[im]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].im.data()[im];

		}

		//transferring data to mimtemplate image type
		mimTemplate[i].image_stride=Finder_CPU.GetmimTemplate().row_stride();
		mimTemplate[i].image_size_y=Finder_CPU.GetmimTemplate().size().y;
		mimTemplate[i].image_size_x=Finder_CPU.GetmimTemplate().size().x;
		for(int im=0;im<mimTemplate[i].image_size_x*mimTemplate[i].image_size_y;im++)
		{
			mimTemplate[i].im[im]=Finder_CPU.GetmimTemplate().data()[im];

		}



		/*Levelg[i].vCornerRowLUT=(int*)malloc(sizeof(int)*mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT.size());
		for(int v=0;v<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT.size();v++)
		{
			Levelg[i].vCornerRowLUT[v]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT[v];
		}

		Levelg[i].vCorners=(cl_float**)malloc(sizeof(cl_float*)*mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.size());
		for(int v=0;v<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners.size();v++)
		{
			Levelg[i].vCorners[v]=(cl_float*)malloc(sizeof(cl_float)*2);
			Levelg[i].vCorners[v][0]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners[v].x;
			Levelg[i].vCorners[v][1]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCorners[v].y;
			 if(v==4&&i==4)
			cout<<"levels loop ref from custom Level x::"<<Levelg[i].vCorners[4][0]<<endl;
		}
		//Levelg[i].vCornerRowLUT=&mCurrentKF.aLevels[Finder_CPU.GetLevel()].vCornerRowLUT[0];

		Levelg[i].vMaxCorners=(cl_float**)malloc(sizeof(cl_float*)*mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners.size());
		for(int v=0;v<mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners.size();v++)
		{
			Levelg[i].vMaxCorners[v]=(cl_float*)malloc(sizeof(cl_float)*2);
			Levelg[i].vMaxCorners[v][0]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[v].x;
			Levelg[i].vMaxCorners[v][1]=mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners[v].y;
		}
*/
		///Levelg[i].vMaxCorners=&mCurrentKF.aLevels[Finder_CPU.GetLevel()].vMaxCorners;


		 //cout<<"vCorners Size"<<L.vCorners.size()<<endl;
		// cout<<"vCornersLUT Size"<<L.vCornerRowLUT.size()<<endl;
		// cout<<"vMAXCorners Size"<<L.vMaxCorners.size()<<endl;




		//cout<<"v2Image["<<i<<"]"<<"["<<j<<"]:"<<v2Image[i][j]<<endl;
		//cout<<"v2Found["<<i<<"]"<<"["<<j<<"]:"<<v2Found[i][j]<<endl;
	}


	//cout<<"assignment successful;!!!"<<endl;
	//cout<<"TD image in cpu.....v2Found 1: "<<TD[4].v2Found[1]<<endl;
	//int k=Imageg[4].im[176+0*640+(104 + 0)];
	// cout<<"image data custom...........: "<<k<<endl;
	//free(image_data);
	//cout<<"size of image type: "<<sizeof(CVD::Image<CVD::byte>);
	/*v2Image = (double **)malloc(vTD.size() * sizeof(double *));

	v2Found = (double **)malloc(vTD.size() * sizeof(double *));

	for( int i=0; i<vTD.size(); i++)
	{
		v2Image[i] =(double *) malloc(2 * sizeof(double));
		v2Found[i] =(double *) malloc(2 * sizeof(double));
	}
	bInImage=(bool *)malloc(vTD.size() * sizeof(bool));
	bPotentiallyVisible=(bool *)malloc(vTD.size() * sizeof(bool));
	bSearched=(bool *)malloc(vTD.size() * sizeof(bool));
	bFound=(bool *)malloc(vTD.size() * sizeof(bool));
	bDidSubPix=(bool *)malloc(vTD.size() * sizeof(bool));

	dSqrtInvNoise=(double *)malloc(vTD.size() * sizeof(double));*/
#endif
	//--------allocating memory ends-----------//

#if(0)//parameter needed for find patch coarse

	//declaring structure of Level type equivalent to Level in patchFineder
	/*typedef struct clImageRef
	{
		int x;
		int y;
	}clImageRef;*/

	typedef struct clLevel
	{

		cl_float** vCorners;
		cl_int image_y;
		cl_int* vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
		cl_int** vMaxCorners;  // The maximal FAST corners
		cl_uchar* im;                // The pyramid level pixels

	}clLevel;

	clLevel* node;

	int* nLevelScale=(int*)malloc(vTD.size() * sizeof(int));
	KeyFrame &mCrntFrm=mCurrentKF;
	Level &L=mCrntFrm.aLevels[2]; //key frames is only 1 so we dont need to declare array.
	int Lim_y=L.im.size().y;
//	int LBegin=L.vCorners.begin();
//	int LEnd=L.vCorners.end();
	int* mnMaxSSD= (int *)malloc(vTD.size() * sizeof(int));
	int* mnSearchLevel=(int *)malloc(vTD.size() * sizeof(int));
	float** mv2CoarsePos;    //this was a vector type in tracker data
	int* vCornerRowLUT=(int *)malloc(L.vCornerRowLUT.size() * sizeof(int));

	for( int i=0; i<vTD.size(); i++)
	{
		TrackerData &TD = *vTD[i];
		PatchFinder &Finder=TD.Finder;

		nLevelScale[i]=1<<Finder.GetLevel(); //mnl>>get this from level helper file and this is from LevelScale Function.
		mnSearchLevel[i]=Finder.GetLevel();
		mnMaxSSD[i]=Finder.mnMaxSSD;
		for(int j=0;j<2;j++)
				{
			mv2CoarsePos[i][j]=Finder.GetCoarsePosAsVector()[0];
			mv2CoarsePos[i][j]=Finder.GetCoarsePosAsVector()[1];

				}
	}
	for( int i=0; i<L.vCornerRowLUT.size(); i++)
	{
		vCornerRowLUT[i]=L.vCornerRowLUT[i]; //mnl>>get this from KeyFRame
	}


#endif

#if(0)//converting values to basic data types
	cout<<"inside ConvertDatatoDevice.........."<<endl;
	for( int i=0; i<vTD.size(); i++)
	{

		for(int j=0;j<2;j++)
		{
/*			cout<<TD.v2Image[0]<<endl;
			cout<<TD.v2Image[1]<<endl;*/
			v2Image[i][j]=vTD[i]->v2Image[j]; //correctly assigned and tested;
			v2Found[i][j]=vTD[i]->v2Found[j];//correctly assigned and tested;
			//cout<<"v2Image["<<i<<"]"<<"["<<j<<"]:"<<v2Image[i][j]<<endl;
			//cout<<"v2Found["<<i<<"]"<<"["<<j<<"]:"<<v2Found[i][j]<<endl;
		}
		bInImage[i]=vTD[i]->bInImage;
		bPotentiallyVisible[i]=vTD[i]->bPotentiallyVisible;
		bSearched[i]=vTD[i]->bSearched;
		bFound[i]=vTD[i]->bFound;
		bDidSubPix[i]=vTD[i]->bDidSubPix;
		dSqrtInvNoise[i]=vTD[i]->dSqrtInvNoise;
	}
#endif
#if(1)//calling kernels
	char* filename="SearchForPoints.cl";
	char* kernelName="SearchForPoints";



	if(vTD.size()>0)
	{
		//SearchForPointsKernel(kernelName,filename,nRange, TD, nFound,vTD.size(),Finder,manMeasAttempted,LEVELS, Imageg,Levelg,mimTemplate,vCornersEndIndex,nSubPixIts	);
	}

	for( int i=0; i<vTD.size(); i++)
	{
		TrackerData &TDForCPU = *vTD[i];
		PatchFinder &Finder_CPU=TDForCPU.Finder;
		TDForCPU.bInImage=TD[i].bInImage;
		TDForCPU.bPotentiallyVisible=TD[i].bPotentiallyVisible;
		TDForCPU.bFound=TD[i].bFound;
		TDForCPU.bSearched=TD[i].bSearched;
		TDForCPU.dSqrtInvNoise=TD[i].dSqrtInvNoise;
		TDForCPU.v2Found[0]=TD[i].v2Found[0];
		TDForCPU.v2Found[1]=TD[i].v2Found[1];
		TDForCPU.bDidSubPix=TD[i].bDidSubPix;
		Finder_CPU.SetmbFound(Finder[i].mbFound);
		Finder_CPU.SetPredicatedPos(Finder[i].mirPredictedPos[0],Finder[i].mirPredictedPos[1]); //its not working
		Finder_CPU.Setmv2CoarsePos(Finder[i].mv2CoarsePos[0],Finder[i].mv2CoarsePos[1]);



		for(int im=0;im<4;im++)
		{
			manMeasAttempted[im]=Finder[i].manMeasAttempted[im];

		}

		for(int im=0;im<4;im++)
		{
			manMeasFound[im]=Finder[i].manMeasFound[im];

		}



	}


#endif

}
//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
  // Which M-estimator are we using?
  int nEstimator = 0;
  static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  if(*gvsEstimator == "Tukey")
    nEstimator = 0;
  else if(*gvsEstimator == "Cauchy")
    nEstimator = 1;
  else if(*gvsEstimator == "Huber")
    nEstimator = 2;
  else 
    {
      cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
      nEstimator = 0;
      *gvsEstimator = "Tukey";
    };
  
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  vector<double> vdErrorSquared;

 // cout<< "Loop teeation at line 1681 tracker.cc vTD.size()"<<vTD.size()<<endl;


  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
	continue;
      TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
      vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
    };
  
  // No valid measurements? Return null update.
  if(vdErrorSquared.size() == 0)
    return makeVector( 0,0,0,0,0,0);
  
  // What is the distribution of errors?
  double dSigmaSquared;
  if(dOverrideSigma > 0)
    dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
  else
    {
      if (nEstimator == 0)
	dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      else if(nEstimator == 1)
	dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else 
	dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
    }
  
  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior


  //cout<< "Loop teeation at line 1716 tracker.cc vTD.size()"<<vTD.size()<<endl;

  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
	continue;
      Vector<2> &v2 = TD.v2Error_CovScaled;
      double dErrorSq = v2 * v2;
      double dWeight;
      
      if(nEstimator == 0)
	dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
	dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else 
	dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
      
      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
	{
	  if(bMarkOutliers)
	    TD.Point.nMEstimatorOutlierCount++;
	  continue;
	}
      else
	if(bMarkOutliers)
	  TD.Point.nMEstimatorInlierCount++;
      
      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
  
  wls.compute();
  return wls.get_mu();
}


// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel()
{
  mse3StartPos = mse3CamFromWorld;
  Vector<6> v6Velocity = mv6CameraVelocity;
  if(mbUseSBIInit)
    {
      v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
      v6Velocity[0] = 0.0;
      v6Velocity[1] = 0.0;
    }
  mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
};


// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel()
{
  SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
  Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);
  Vector<6> v6OldVel = mv6CameraVelocity;
  
  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
  mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);
  
  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  Vector<6> v6 = mv6CameraVelocity;
  v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

// Time to add a new keyframe? The MapMaker handles most of this.
void Tracker::AddNewKeyFrame()
{
  mMapMaker.AddKeyFrame(mCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
}

// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality()
{
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;
  
  for(int i=0; i<LEVELS; i++)
    {
      nTotalAttempted += manMeasAttempted[i];
      nTotalFound += manMeasFound[i];
      if(i>=2) nLargeAttempted += manMeasAttempted[i];
      if(i>=2) nLargeFound += manMeasFound[i];
    }
  
  if(nTotalFound == 0 || nTotalAttempted == 0)
    mTrackingQuality = BAD;
  else
    {
      double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
      double dLargeFracFound;
      if(nLargeAttempted > 10)
	dLargeFracFound = (double) nLargeFound / nLargeAttempted;
      else
	dLargeFracFound = dTotalFracFound;

      static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
      static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);
      
      
      if(dTotalFracFound > *gvdQualityGood)
	mTrackingQuality = GOOD;
      else if(dLargeFracFound < *gvdQualityLost)
	mTrackingQuality = BAD;
      else
	mTrackingQuality = DODGY;
    }
  
  if(mTrackingQuality == DODGY)
    {
      // Further heuristics to see if it's actually bad, not just dodgy...
      // If the camera pose estimate has run miles away, it's probably bad.
      if(mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
	mTrackingQuality = BAD;
    }
  
  if(mTrackingQuality==BAD)
    mnLostFrames++;
  else
    mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
  return mMessageForUser.str();
}

void Tracker::CalcSBIRotation()
{
  mpSBILastFrame->MakeJacs();
  pair<SE2<>, double> result_pair;
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  mv6SBIRot = se3Adjust.ln();
}

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here








