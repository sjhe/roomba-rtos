/*
 * error.h
 *
 *  Created on: Jul 7, 2013
 *      Author: andpol
 */

#ifndef ERROR_H_
#define ERROR_H_

typedef enum {
	ERRNO_NO_ERROR = 0,
	ERRNO_EXCEEDS_MAX_EVENT,
	ERRNO_PERIODIC_CALLED_WAIT,
	ERRNO_EXCEEDS_MAX_PROCS,
	ERRNO_INVALID_PROC_TYPE,
	ERRNO_INVALID_KERNEL_REQ,
	ERRNO_PERIODIC_TASK_OVERLAP,
	ERRNO_PERIODIC_TASK_EXCEEDS_WCET,
	ERRNO_MULTIPLE_TASKS_CALLED_WAIT,

	ERRNO_UNKNOWN_ERR,
} ERRNO;

ERRNO errno;

// Called on OS_Abort() if not NULL.
void (*os_err_handler)(void);

#endif /* ERROR_H_ */
