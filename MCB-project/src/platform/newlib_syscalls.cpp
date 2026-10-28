// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#ifndef PLATFORM_HOSTED
#include <cerrno>
#include <cstddef>
#include <sys/stat.h>
#include <sys/types.h>

// The board has no POSIX files, processes, or entropy source.
// Hardware I/O uses Taproot directly; unsupported libc calls must report failure.
namespace {
int unsupported() {
    errno = ENOSYS;
    return -1;
}
}
extern "C" {
int _close(int) { return unsupported(); }
int _fstat(int, struct stat*) { return unsupported(); }
pid_t _getpid() { return 1; }
int _isatty(int) { unsupported(); return 0; }
int _kill(int, int) { return unsupported(); }
off_t _lseek(int, off_t, int) { return unsupported(); }
ssize_t _read(int, void*, size_t) { return unsupported(); }
ssize_t _write(int, const void*, size_t) { return unsupported(); }
int _getentropy(void*, size_t) { return unsupported(); }
}
#endif
