#ifndef _PRINTK_SYSLOG_H
#define _PRINTK_SYSLOG_H

#include <linux/syscalls.h>
#include <linux/syslog.h>

/* the next printk record to read by syslog(READ) or /proc/kmsg */
extern u64 printk_syslog_seq;
extern u32 printk_syslog_idx;
extern enum printk_log_flags printk_syslog_prev;

#endif
