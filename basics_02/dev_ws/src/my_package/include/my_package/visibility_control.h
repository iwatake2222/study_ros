#ifndef MY_PACKAGE__VISIBILITY_CONTROL_H_
#define MY_PACKAGE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MY_PACKAGE_EXPORT __attribute__ ((dllexport))
    #define MY_PACKAGE_IMPORT __attribute__ ((dllimport))
  #else
    #define MY_PACKAGE_EXPORT __declspec(dllexport)
    #define MY_PACKAGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MY_PACKAGE_BUILDING_DLL
    #define MY_PACKAGE_PUBLIC MY_PACKAGE_EXPORT
  #else
    #define MY_PACKAGE_PUBLIC MY_PACKAGE_IMPORT
  #endif
  #define MY_PACKAGE_PUBLIC_TYPE MY_PACKAGE_PUBLIC
  #define MY_PACKAGE_LOCAL
#else
  #define MY_PACKAGE_EXPORT __attribute__ ((visibility("default")))
  #define MY_PACKAGE_IMPORT
  #if __GNUC__ >= 4
    #define MY_PACKAGE_PUBLIC __attribute__ ((visibility("default")))
    #define MY_PACKAGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MY_PACKAGE_PUBLIC
    #define MY_PACKAGE_LOCAL
  #endif
  #define MY_PACKAGE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MY_PACKAGE__VISIBILITY_CONTROL_H_