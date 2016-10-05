#pragma once

#ifdef __cplusplus
extern "C" {
#endif
    int chvsnprintf(char *str, size_t size, const char *fmt, va_list ap);
    int printf(const char *fmt, ...);
#ifdef __cplusplus
}
#endif
