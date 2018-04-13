#pragma once

#ifndef ZEKKU_BASE_H
#define ZEKKU_BASE_H

#ifndef __cplusplus
#error "This is a C++ library, not a C library!"
#endif

#if __cplusplus < 201402L
#error "C++14 or higher is required"
#endif

#if __cplusplus >= 201703L
#define ZK_CPP17 1
#endif

#if __cplusplus > 201703L
#define ZK_CPP20 1
#endif

#ifdef ZK_CPP20
#define ZK_NOUNIQADDR [[no_unique_address]]
#else
#define ZK_NOUNIQADDR
#endif

#ifndef ZK_CPP17
#define ZK_REGISTER register
#else
#define ZK_REGISTER
#endif

#ifdef __GNUC__
#define ZK_RESTRICT __restrict__
#elif _MSC_VER
#define ZK_RESTRICT __restrict
#else
#define ZK_RESTRICT
#endif

#endif