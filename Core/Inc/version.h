/**
 * Contains current firmware version.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#ifndef MATRIX4_MUEB_FW_INC_VERSION_H_
#define MATRIX4_MUEB_FW_INC_VERSION_H_

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

#endif  // MATRIX4_MUEB_FW_INC_VERSION_H_
