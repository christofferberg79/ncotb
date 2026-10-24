#pragma once

#include <raylib.h>

// macOS's Cmd and Option are Ctrl and Alt elsewhere (on Windows, Win+S opens the
// system search). Macros, so labels can be built by string literal concatenation.
#ifdef __APPLE__
#define NCOTB_CMD "Cmd"
#define NCOTB_OPTION "Option"
#else
#define NCOTB_CMD "Ctrl"
#define NCOTB_OPTION "Alt"
#endif

namespace ncotb::app {

/// Cmd on macOS, Ctrl elsewhere: the modifier for shortcuts like Cmd+S.
inline bool cmdKeyDown() {
#ifdef __APPLE__
    return IsKeyDown(KEY_LEFT_SUPER) || IsKeyDown(KEY_RIGHT_SUPER);
#else
    return IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
#endif
}

} // namespace ncotb::app
