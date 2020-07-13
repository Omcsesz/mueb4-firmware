/*
 * version.hpp
 *
 *  Created on: May 30, 2018
 *      Author: kisada
 */

#pragma once

const char* mueb_version =

    "v1.0"

#ifdef _COMMIT
    "-" _COMMIT
#endif
// TODO else compile error

#ifdef _DIRTYTREE
    "-dirty"
#endif

    ;
