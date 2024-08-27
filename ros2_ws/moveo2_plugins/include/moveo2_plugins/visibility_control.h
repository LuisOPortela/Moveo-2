#ifndef MOVEO2_PLUGINS__VISIBILITY_CONTROL_H_
#define MOVEO2_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEO2_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define MOVEO2_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEO2_PLUGINS_EXPORT __declspec(dllexport)
    #define MOVEO2_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEO2_PLUGINS_BUILDING_LIBRARY
    #define MOVEO2_PLUGINS_PUBLIC MOVEO2_PLUGINS_EXPORT
  #else
    #define MOVEO2_PLUGINS_PUBLIC MOVEO2_PLUGINS_IMPORT
  #endif
  #define MOVEO2_PLUGINS_PUBLIC_TYPE MOVEO2_PLUGINS_PUBLIC
  #define MOVEO2_PLUGINS_LOCAL
#else
  #define MOVEO2_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define MOVEO2_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define MOVEO2_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEO2_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEO2_PLUGINS_PUBLIC
    #define MOVEO2_PLUGINS_LOCAL
  #endif
  #define MOVEO2_PLUGINS_PUBLIC_TYPE
#endif

#endif  // MOVEO2_PLUGINS__VISIBILITY_CONTROL_H_
