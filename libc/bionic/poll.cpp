/*
 * Copyright (C) 2013 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/poll.h>

#include "private/kernel_sigset_t.h"

extern "C" int __ppoll(pollfd*, unsigned int, timespec*, const kernel_sigset_t*, size_t);

void timespec_from_ms(timespec& ts, const int ms) {
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000;
}

int poll(pollfd* fds, nfds_t fd_count, int ms) {
  timespec ts;
  timespec* ts_ptr = NULL;
  if (ms >= 0) {
    timespec_from_ms(ts, ms);
    ts_ptr = &ts;
  }
  return __ppoll(fds, fd_count, ts_ptr, NULL, 0);
}

int ppoll(pollfd* fds, nfds_t fd_count, const timespec* ts, const sigset_t* ss) {
  timespec mutable_ts;
  timespec* mutable_ts_ptr = NULL;
  if (ts != NULL) {
    mutable_ts = *ts;
    mutable_ts_ptr = &mutable_ts;
  }

  kernel_sigset_t kernel_ss;
  kernel_sigset_t* kernel_ss_ptr = NULL;
  if (ss != NULL) {
    kernel_ss.set(ss);
    kernel_ss_ptr = &kernel_ss;
  }

  return __ppoll(fds, fd_count, mutable_ts_ptr, kernel_ss_ptr, sizeof(kernel_ss));
}
