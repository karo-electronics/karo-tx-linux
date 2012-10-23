#ifndef _PRINTK_LOG_H
#define _PRINTK_LOG_H

/*
 * The printk log buffer consists of a chain of concatenated variable
 * length records. Every record starts with a record header, containing
 * the overall length of the record.
 *
 * The heads to the first and last entry in the buffer, as well as the
 * sequence numbers of these both entries are maintained when messages
 * are stored..
 *
 * If the heads indicate available messages, the length in the header
 * tells the start next message. A length == 0 for the next message
 * indicates a wrap-around to the beginning of the buffer.
 *
 * Every record carries the monotonic timestamp in microseconds, as well as
 * the standard userspace syslog level and syslog facility. The usual
 * kernel messages use LOG_KERN; userspace-injected messages always carry
 * a matching syslog facility, by default LOG_USER. The origin of every
 * message can be reliably determined that way.
 *
 * The human readable log message directly follows the message header. The
 * length of the message text is stored in the header, the stored message
 * is not terminated.
 *
 * Optionally, a message can carry a dictionary of properties (key/value pairs),
 * to provide userspace with a machine-readable message context.
 *
 * Examples for well-defined, commonly used property names are:
 *   DEVICE=b12:8               device identifier
 *                                b12:8         block dev_t
 *                                c127:3        char dev_t
 *                                n8            netdev ifindex
 *                                +sound:card0  subsystem:devname
 *   SUBSYSTEM=pci              driver-core subsystem name
 *
 * Valid characters in property names are [a-zA-Z0-9.-_]. The plain text value
 * follows directly after a '=' character. Every property is terminated by
 * a '\0' character. The last property is not terminated.
 *
 * Example of a message structure:
 *   0000  ff 8f 00 00 00 00 00 00      monotonic time in nsec
 *   0008  34 00                        record is 52 bytes long
 *   000a        0b 00                  text is 11 bytes long
 *   000c              1f 00            dictionary is 23 bytes long
 *   000e                    03 00      LOG_KERN (facility) LOG_ERR (level)
 *   0010  69 74 27 73 20 61 20 6c      "it's a l"
 *         69 6e 65                     "ine"
 *   001b           44 45 56 49 43      "DEVIC"
 *         45 3d 62 38 3a 32 00 44      "E=b8:2\0D"
 *         52 49 56 45 52 3d 62 75      "RIVER=bu"
 *         67                           "g"
 *   0032     00 00 00                  padding to next message header
 *
 * The 'struct printk_log' buffer header must never be directly exported to
 * userspace, it is a kernel-private implementation detail that might
 * need to be changed in the future, when the requirements change.
 *
 * /dev/kmsg exports the structured data in the following line format:
 *   "level,sequnum,timestamp;<message text>\n"
 *
 * The optional key/value pairs are attached as continuation lines starting
 * with a space character and terminated by a newline. All possible
 * non-prinatable characters are escaped in the "\xff" notation.
 *
 * Users of the export format should ignore possible additional values
 * separated by ',', and find the message after the ';' character.
 */

enum printk_log_flags {
	LOG_NOCONS	= 1,	/* already flushed, do not print to console */
	LOG_NEWLINE	= 2,	/* text ended with a newline */
	LOG_PREFIX	= 4,	/* text started with a prefix */
	LOG_CONT	= 8,	/* text is a fragment of a continuation line */
};

struct printk_log {
	u64 ts_nsec;		/* timestamp in nanoseconds */
	u16 len;		/* length of entire record */
	u16 text_len;		/* length of text buffer */
	u16 dict_len;		/* length of dictionary buffer */
	u8 facility;		/* syslog facility */
	u8 flags:5;		/* internal record flags */
	u8 level:3;		/* syslog level */
};

#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
#define PRINTK_LOG_ALIGN 4
#else
#define PRINTK_LOG_ALIGN __alignof__(struct printk_log)
#endif
#define __PRINTK_LOG_BUF_LEN (1 << CONFIG_LOG_BUF_SHIFT)

extern raw_spinlock_t printk_logbuf_lock;
extern wait_queue_head_t printk_log_wait;
extern u64 printk_log_first_seq;
extern u32 printk_log_first_idx;
extern u64 printk_log_next_seq;
extern u32 printk_log_next_idx;
extern u64 printk_log_clear_seq;
extern u32 printk_log_clear_idx;
extern char __printk_log_buf[__PRINTK_LOG_BUF_LEN] __aligned(PRINTK_LOG_ALIGN);
extern char *printk_log_buf;
extern u32 printk_log_buf_len;

char *printk_log_text(const struct printk_log *msg);
char *printk_log_dict(const struct printk_log *msg);
struct printk_log *printk_log_from_idx(u32 idx);
u32 printk_log_next(u32 idx);
void printk_log_store(int facility, int level,
		      enum printk_log_flags flags, u64 ts_nsec,
		      const char *dict, u16 dict_len,
		      const char *text, u16 text_len);

#endif
