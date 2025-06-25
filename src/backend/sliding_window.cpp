#include "backend/sliding_window.h"

namespace backend {

void SlidingWindow::clearSlidingWindow() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    sliding_window[i].clear();
  }
}

}  // namespace backend