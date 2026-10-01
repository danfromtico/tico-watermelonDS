#pragma once
#define PROP_VALUE_MAX 92
static inline int __system_property_get(const char*, char* v) { v[0] = 0; return 0; }
