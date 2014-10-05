#ifndef SSB_COMMON_ENUM_H_
#define SSB_COMMON_ENUM_H_

namespace ssb_common_enum {

enum Config { DEBUG,
              LEFT,
              RIGHT };

enum PlayState { STOP,
                 STARTPLAY,
                 ONPLAY,
                 PLAY2PAUSE,
                 ONPAUSE,
                 PAUSE2PLAY,
                 EDIT,
                 REPLAY,
                 NONE };

} // ssb_common_enum

#endif
