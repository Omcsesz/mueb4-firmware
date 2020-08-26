/**
 * Contains current firmware version.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#pragma once

const char* mueb_version =

    "v2.1"

#ifdef _COMMIT
    "-" _COMMIT
#endif
// TODO else compile error

#ifdef _DIRTYTREE
    "-dirty"
#endif

    ;
