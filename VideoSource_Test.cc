// Copyright 2008 Isis Innovation Limited
#include "VideoSource.h"
#include <cvd/colourspace_convert.h>
#include <cvd/videofilebuffer.h>
#include <cvd/videoframe.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource::VideoSource()
{
  cout << "  VideoSource_Linux: Opening video source..." << endl;
  
  VideoFileBuffer<Rgb<byte> > * pvb = new VideoFileBuffer<Rgb<byte> > ("video.mpeg");
  pvb->on_end_of_buffer(CVD::VideoBufferFlags::Loop);
  cout << pvb->frame_rate() << endl;

  mirSize = pvb -> size();
  mptr = pvb;

  cout << "  ... got video source." << endl;
};

ImageRef VideoSource::Size()
{ 
  return mirSize;
};

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  VideoFileBuffer<Rgb<byte> >* pvb = (VideoFileBuffer<Rgb<byte> >*) mptr;
  VideoFileFrame<Rgb<byte> > *pVidFrame = pvb->get_frame();
  convert_image(*pVidFrame, imBW);
  convert_image(*pVidFrame, imRGB);
  pvb->put_frame(pVidFrame);
}
