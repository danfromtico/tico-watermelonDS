// Switch stand-in for the header libslirp's build generates.
#pragma once

#define SLIRP_MAJOR_VERSION 4
#define SLIRP_MINOR_VERSION 8
#define SLIRP_MICRO_VERSION 0
#define SLIRP_VERSION_STRING "4.8.0"

#define SLIRP_CHECK_VERSION(major, minor, micro) \
    (SLIRP_MAJOR_VERSION > (major) || \
     (SLIRP_MAJOR_VERSION == (major) && SLIRP_MINOR_VERSION > (minor)) || \
     (SLIRP_MAJOR_VERSION == (major) && SLIRP_MINOR_VERSION == (minor) && SLIRP_MICRO_VERSION >= (micro)))
