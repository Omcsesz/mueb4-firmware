/**
 * Contains current firmware version.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#pragma once

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

const char* mueb_version =

    "v2.1"

#ifdef _COMMIT
    "-" STR(_COMMIT)
#endif

#ifdef _DIRTYTREE
        "-dirty"
#endif

    ;
