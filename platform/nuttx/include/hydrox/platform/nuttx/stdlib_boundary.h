#pragma once

// The GNU libstdc++ <cstdlib> wrapper uses include_next and otherwise reaches
// newlib's C library after NuttX's C library has already been selected. Load
// the NuttX declarations first, then suppress that second newlib stdlib pass.
#include <stdlib.h>
#define _STDLIB_H_
