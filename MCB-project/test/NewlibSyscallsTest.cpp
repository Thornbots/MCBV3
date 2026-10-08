// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>

extern "C" {
int _close(int);
int _fstat(int, struct stat*);
pid_t _getpid();
int _isatty(int);
int _kill(int, int);
off_t _lseek(int, off_t, int);
ssize_t _read(int, void*, size_t);
ssize_t _write(int, const void*, size_t);
int _getentropy(void*, size_t);
}

TEST(NewlibSyscallsTest, UnsupportedCallsReportFailure) {
    char buffer[4]{};
    struct stat status{};
    const auto fails = [](auto call) {
        errno = 0;
        EXPECT_EQ(call(), -1);
        EXPECT_EQ(errno, ENOSYS);
    };
    fails([] { return _close(0); });
    fails([&] { return _fstat(0, &status); });
    fails([] { return _kill(1, 0); });
    fails([] { return _lseek(0, 0, 0); });
    fails([&] { return _read(0, buffer, sizeof(buffer)); });
    fails([&] { return _write(1, buffer, sizeof(buffer)); });
    fails([&] { return _getentropy(buffer, sizeof(buffer)); });
}

TEST(NewlibSyscallsTest, BoardIsOneProcessWithoutTerminals) {
    EXPECT_EQ(_getpid(), 1);
    errno = 0;
    EXPECT_EQ(_isatty(0), 0);
    EXPECT_EQ(errno, ENOSYS);
}
