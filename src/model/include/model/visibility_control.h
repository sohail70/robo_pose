#ifndef MODEL__VISIBILITY_CONTROL_H_
#define MODEL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MODEL_EXPORT __attribute__ ((dllexport))
    #define MODEL_IMPORT __attribute__ ((dllimport))
  #else
    #define MODEL_EXPORT __declspec(dllexport)
    #define MODEL_IMPORT __declspec(dllimport)
  #endif
  #ifdef MODEL_BUILDING_LIBRARY
    #define MODEL_PUBLIC MODEL_EXPORT
  #else
    #define MODEL_PUBLIC MODEL_IMPORT
  #endif
  #define MODEL_PUBLIC_TYPE MODEL_PUBLIC
  #define MODEL_LOCAL
#else
  #define MODEL_EXPORT __attribute__ ((visibility("default")))
  #define MODEL_IMPORT
  #if __GNUC__ >= 4
    #define MODEL_PUBLIC __attribute__ ((visibility("default")))
    #define MODEL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MODEL_PUBLIC
    #define MODEL_LOCAL
  #endif
  #define MODEL_PUBLIC_TYPE
#endif

#endif  // MODEL__VISIBILITY_CONTROL_H_
