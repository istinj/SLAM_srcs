#include "lk_tracker.h"
#include "rgbdt/FrameInfoMessage.h"
#include "rgbdt/PointTrackMessage.h"

namespace rgbdt {

  void msg2frameInfo (FrameInfo& dest, const FrameInfoMessage& src);
  void frameInfo2Msg (FrameInfoMessage& dest, const FrameInfo& src);

}
