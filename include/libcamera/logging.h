/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * logging.h - Logging infrastructure
 */
#ifndef __LIBCAMERA_LOGGING_H__
#define __LIBCAMERA_LOGGING_H__

namespace libcamera {

enum LoggingTarget {
	LoggingTargetNone,
	LoggingTargetSyslog,
	LoggingTargetFile,
	LoggingTargetStream,
};

int logSetFile(const char *path);
int logSetStream(std::ostream *stream);
int logSetTarget(LoggingTarget target);
void logSetLevel(const char *category, const char *level);
void backtrace();

} /* namespace libcamera */

#endif /* __LIBCAMERA_LOGGING_H__ */
