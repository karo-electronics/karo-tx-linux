#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/kexec.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>

#include "printk_log.h"

/*
 * The printk_logbuf_lock protects kmsg buffer, indices, counters. It is also
 * used in interesting ways to provide interlocking in console_unlock();
 */
DEFINE_RAW_SPINLOCK(printk_logbuf_lock);

#ifdef CONFIG_PRINTK

/* index and sequence number of the first record stored in the buffer */
u64 printk_log_first_seq;
u32 printk_log_first_idx;

/* index and sequence number of the next record to store in the buffer */
u64 printk_log_next_seq;
u32 printk_log_next_idx;

/* the next printk record to read after the last 'clear' command */
u64 printk_log_clear_seq;
u32 printk_log_clear_idx;

/* record buffer */
char __printk_log_buf[__PRINTK_LOG_BUF_LEN] __aligned(PRINTK_LOG_ALIGN);
char *printk_log_buf = __printk_log_buf;
u32 printk_log_buf_len = __PRINTK_LOG_BUF_LEN;

/* human readable text of the record */
char *printk_log_text(const struct printk_log *msg)
{
	return (char *)msg + sizeof(struct printk_log);
}

/* optional key/value pair dictionary attached to the record */
char *printk_log_dict(const struct printk_log *msg)
{
	return (char *)msg + sizeof(struct printk_log) + msg->text_len;
}

/* get record by index; idx must point to valid msg */
struct printk_log *printk_log_from_idx(u32 idx)
{
	struct printk_log *msg = (struct printk_log *)(printk_log_buf + idx);

	/*
	 * A length == 0 record is the end of buffer marker. Wrap around and
	 * read the message at the start of the buffer.
	 */
	if (!msg->len)
		return (struct printk_log *)printk_log_buf;
	return msg;
}

/* get next record; idx must point to valid msg */
u32 printk_log_next(u32 idx)
{
	struct printk_log *msg = (struct printk_log *)(printk_log_buf + idx);

	/* length == 0 indicates the end of the buffer; wrap */
	/*
	 * A length == 0 record is the end of buffer marker. Wrap around and
	 * read the message at the start of the buffer as *this* one, and
	 * return the one after that.
	 */
	if (!msg->len) {
		msg = (struct printk_log *)printk_log_buf;
		return msg->len;
	}
	return idx + msg->len;
}

/* insert record into the buffer, discard old ones, update heads */
void printk_log_store(int facility, int level,
		      enum printk_log_flags flags, u64 ts_nsec,
		      const char *dict, u16 dict_len,
		      const char *text, u16 text_len)
{
	struct printk_log *msg;
	u32 size, pad_len;

	/* number of '\0' padding bytes to next message */
	size = sizeof(struct printk_log) + text_len + dict_len;
	pad_len = (-size) & (PRINTK_LOG_ALIGN - 1);
	size += pad_len;

	while (printk_log_first_seq < printk_log_next_seq) {
		u32 free;

		if (printk_log_next_idx > printk_log_first_idx)
			free = max(printk_log_buf_len - printk_log_next_idx, printk_log_first_idx);
		else
			free = printk_log_first_idx - printk_log_next_idx;

		if (free > size + sizeof(struct printk_log))
			break;

		/* drop old messages until we have enough contiuous space */
		printk_log_first_idx = printk_log_next(printk_log_first_idx);
		printk_log_first_seq++;
	}

	if (printk_log_next_idx + size + sizeof(struct printk_log) >= printk_log_buf_len) {
		/*
		 * This message + an additional empty header does not fit
		 * at the end of the buffer. Add an empty header with len == 0
		 * to signify a wrap around.
		 */
		memset(printk_log_buf + printk_log_next_idx, 0, sizeof(struct printk_log));
		printk_log_next_idx = 0;
	}

	/* fill message */
	msg = (struct printk_log *)(printk_log_buf + printk_log_next_idx);
	memcpy(printk_log_text(msg), text, text_len);
	msg->text_len = text_len;
	memcpy(printk_log_dict(msg), dict, dict_len);
	msg->dict_len = dict_len;
	msg->facility = facility;
	msg->level = level & 7;
	msg->flags = flags & 0x1f;
	if (ts_nsec > 0)
		msg->ts_nsec = ts_nsec;
	else
		msg->ts_nsec = local_clock();
	memset(printk_log_dict(msg) + dict_len, 0, pad_len);
	msg->len = sizeof(struct printk_log) + text_len + dict_len + pad_len;

	/* insert message */
	printk_log_next_idx += msg->len;
	printk_log_next_seq++;
}

#if defined(CONFIG_PRINTK_TIME)
static bool printk_time = 1;
#else
static bool printk_time;
#endif
module_param_named(time, printk_time, bool, S_IRUGO | S_IWUSR);

size_t printk_print_time(u64 ts, char *buf)
{
	unsigned long rem_nsec;

	if (!printk_time)
		return 0;

	if (!buf)
		return 15;

	rem_nsec = do_div(ts, 1000000000);
	return sprintf(buf, "[%5lu.%06lu] ",
		       (unsigned long)ts, rem_nsec / 1000);
}

static size_t print_prefix(const struct printk_log *msg, bool syslog, char *buf)
{
	size_t len = 0;
	unsigned int prefix = (msg->facility << 3) | msg->level;

	if (syslog) {
		if (buf) {
			len += sprintf(buf, "<%u>", prefix);
		} else {
			len += 3;
			if (prefix > 999)
				len += 3;
			else if (prefix > 99)
				len += 2;
			else if (prefix > 9)
				len++;
		}
	}

	len += printk_print_time(msg->ts_nsec, buf ? buf + len : NULL);
	return len;
}

size_t printk_msg_print_text(const struct printk_log *msg,
			     enum printk_log_flags prev,
			     bool syslog, char *buf, size_t size)
{
	const char *text = printk_log_text(msg);
	size_t text_size = msg->text_len;
	bool prefix = true;
	bool newline = true;
	size_t len = 0;

	if ((prev & LOG_CONT) && !(msg->flags & LOG_PREFIX))
		prefix = false;

	if (msg->flags & LOG_CONT) {
		if ((prev & LOG_CONT) && !(prev & LOG_NEWLINE))
			prefix = false;

		if (!(msg->flags & LOG_NEWLINE))
			newline = false;
	}

	do {
		const char *next = memchr(text, '\n', text_size);
		size_t text_len;

		if (next) {
			text_len = next - text;
			next++;
			text_size -= next - text;
		} else {
			text_len = text_size;
		}

		if (buf) {
			if (print_prefix(msg, syslog, NULL) +
			    text_len + 1 >= size - len)
				break;

			if (prefix)
				len += print_prefix(msg, syslog, buf + len);
			memcpy(buf + len, text, text_len);
			len += text_len;
			if (next || newline)
				buf[len++] = '\n';
		} else {
			/* SYSLOG_ACTION_* buffer size only calculation */
			if (prefix)
				len += print_prefix(msg, syslog, NULL);
			len += text_len;
			if (next || newline)
				len++;
		}

		prefix = true;
		text = next;
	} while (text);

	return len;
}

#else /* CONFIG_PRINTK */

#define LOG_LINE_MAX		0
#define PREFIX_MAX		0
#define LOG_LINE_MAX 0
u64 printk_log_first_seq;
u32 printk_log_first_idx;
u64 printk_log_next_seq;
struct printk_log *printk_log_from_idx(u32 idx) { return NULL; }
u32 printk_log_next(u32 idx) { return 0; }
size_t printk_print_time(u64 ts, char *buf) { return 0; }
size_t printk_msg_print_text(const struct printk_log *msg,
			     enum printk_log_flags prev,
			     bool syslog, char *buf, size_t size)
{
	return 0;
}

#endif /* CONFIG_PRINTK */
