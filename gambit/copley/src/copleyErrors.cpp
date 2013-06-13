#include "copley/copleyErrors.h"

static const char *copleyErrorStrings[16] = {
    "No error",
    "Failed socket",
    "Failed setsockopt",
    "Timeout",
    "Unexpected COB ID",
    "Bad SCS",
    "Not expedited",
    "Bad index",
    "Bad subindex",
    "Bad data",
    "Bad CCS",
    "Bad length",
    "Too fast",
    "Too slow",
    "Home not found",
    "Move not active"
};

const char *copleyGetErrorString(int code) {
    if(code < 0 || code > 15) 
        return "Unknown error";
    return copleyErrorStrings[code];
}

