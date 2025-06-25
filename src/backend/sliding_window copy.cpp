#include "sliding_window.h"

void SlidingWindow::clearSlidingWindow() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        sliding_window[i].clear();
    }
}