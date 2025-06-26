#ifndef COMMON__COMMON_TYPES_H
#define COMMON__COMMON_TYPES_H

namespace common {

enum SolverFlag { INITIAL = 0, NON_LINEAR = 1 };
enum MarginalizationFlag { MARGIN_OLD_KEYFRAME = 0, MARGIN_NEW_GENERAL_FRAME = 1 };

} // namespace common

#endif  // COMMON__COMMON_TYPES_H