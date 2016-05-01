#ifndef DBOW2API_H_
#define DBOW2API_H_

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef DBOW2_SHARED_LIBS
#ifdef DBoW2_EXPORTS
#define DBOW2_API __declspec(dllexport)
#else
#define DBOW2_API __declspec(dllimport)
#endif
#else
#define DBOW2_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define DBOW2_API
#endif

#endif // DBOW2API_H_