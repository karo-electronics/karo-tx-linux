#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <linux/syslog.h>

#include "printk_log.h"
#include "printk_syslog.h"

/* the next printk record to read by syslog(READ) or /proc/kmsg */
u64 printk_syslog_seq;
u32 printk_syslog_idx;
enum printk_log_flags printk_syslog_prev;

#ifdef CONFIG_PRINTK

static size_t printk_syslog_partial;

#ifdef CONFIG_SECURITY_DMESG_RESTRICT
int dmesg_restrict = 1;
#else
int dmesg_restrict;
#endif

static int printk_syslog_action_restricted(int type)
{
	if (dmesg_restrict)
		return 1;
	/* Unless restricted, we allow "read all" and "get buffer size" for everybody */
	return type != SYSLOG_ACTION_READ_ALL && type != SYSLOG_ACTION_SIZE_BUFFER;
}

static int check_syslog_permissions(int type, bool from_file)
{
	/*
	 * If this is from /proc/kmsg and we've already opened it, then we've
	 * already done the capabilities checks at open time.
	 */
	if (from_file && type != SYSLOG_ACTION_OPEN)
		return 0;

	if (printk_syslog_action_restricted(type)) {
		if (capable(CAP_SYSLOG))
			return 0;
		/* For historical reasons, accept CAP_SYS_ADMIN too, with a warning */
		if (capable(CAP_SYS_ADMIN)) {
			printk_once(KERN_WARNING "%s (%d): "
				 "Attempt to access syslog with CAP_SYS_ADMIN "
				 "but no CAP_SYSLOG (deprecated).\n",
				 current->comm, task_pid_nr(current));
			return 0;
		}
		return -EPERM;
	}
	return 0;
}

static int printk_syslog_print(char __user *buf, int size)
{
	char *text;
	struct printk_log *msg;
	int len = 0;

	text = kmalloc(PRINTK_LOG_LINE_MAX + PRINTK_PREFIX_MAX, GFP_KERNEL);
	if (!text)
		return -ENOMEM;

	while (size > 0) {
		size_t n;
		size_t skip;

		raw_spin_lock_irq(&printk_logbuf_lock);
		if (printk_syslog_seq < printk_log_first_seq) {
			/* messages are gone, move to first one */
			printk_syslog_seq = printk_log_first_seq;
			printk_syslog_idx = printk_log_first_idx;
			printk_syslog_prev = 0;
			printk_syslog_partial = 0;
		}
		if (printk_syslog_seq == printk_log_next_seq) {
			raw_spin_unlock_irq(&printk_logbuf_lock);
			break;
		}

		skip = printk_syslog_partial;
		msg = printk_log_from_idx(printk_syslog_idx);
		n = printk_msg_print_text(msg, printk_syslog_prev, true, text,
					  PRINTK_LOG_LINE_MAX + PRINTK_PREFIX_MAX);
		if (n - printk_syslog_partial <= size) {
			/* message fits into buffer, move forward */
			printk_syslog_idx = printk_log_next(printk_syslog_idx);
			printk_syslog_seq++;
			printk_syslog_prev = msg->flags;
			n -= printk_syslog_partial;
			printk_syslog_partial = 0;
		} else if (!len){
			/* partial read(), remember position */
			n = size;
			printk_syslog_partial += n;
		} else
			n = 0;
		raw_spin_unlock_irq(&printk_logbuf_lock);

		if (!n)
			break;

		if (copy_to_user(buf, text + skip, n)) {
			if (!len)
				len = -EFAULT;
			break;
		}

		len += n;
		size -= n;
		buf += n;
	}

	kfree(text);
	return len;
}

static int printk_syslog_print_all(char __user *buf, int size, bool clear)
{
	char *text;
	int len = 0;

	text = kmalloc(PRINTK_LOG_LINE_MAX + PRINTK_PREFIX_MAX, GFP_KERNEL);
	if (!text)
		return -ENOMEM;

	raw_spin_lock_irq(&printk_logbuf_lock);
	if (buf) {
		u64 next_seq;
		u64 seq;
		u32 idx;
		enum printk_log_flags prev;

		if (printk_log_clear_seq < printk_log_first_seq) {
			/* messages are gone, move to first available one */
			printk_log_clear_seq = printk_log_first_seq;
			printk_log_clear_idx = printk_log_first_idx;
		}

		/*
		 * Find first record that fits, including all following records,
		 * into the user-provided buffer for this dump.
		 */
		seq = printk_log_clear_seq;
		idx = printk_log_clear_idx;
		prev = 0;
		while (seq < printk_log_next_seq) {
			struct printk_log *msg = printk_log_from_idx(idx);

			len += printk_msg_print_text(msg, prev, true, NULL, 0);
			prev = msg->flags;
			idx = printk_log_next(idx);
			seq++;
		}

		/* move first record forward until length fits into the buffer */
		seq = printk_log_clear_seq;
		idx = printk_log_clear_idx;
		prev = 0;
		while (len > size && seq < printk_log_next_seq) {
			struct printk_log *msg = printk_log_from_idx(idx);

			len -= printk_msg_print_text(msg, prev, true, NULL, 0);
			prev = msg->flags;
			idx = printk_log_next(idx);
			seq++;
		}

		/* last message fitting into this dump */
		next_seq = printk_log_next_seq;

		len = 0;
		prev = 0;
		while (len >= 0 && seq < next_seq) {
			struct printk_log *msg = printk_log_from_idx(idx);
			int textlen;

			textlen = printk_msg_print_text(msg, prev, true, text,
							PRINTK_LOG_LINE_MAX + PRINTK_PREFIX_MAX);
			if (textlen < 0) {
				len = textlen;
				break;
			}
			idx = printk_log_next(idx);
			seq++;
			prev = msg->flags;

			raw_spin_unlock_irq(&printk_logbuf_lock);
			if (copy_to_user(buf + len, text, textlen))
				len = -EFAULT;
			else
				len += textlen;
			raw_spin_lock_irq(&printk_logbuf_lock);

			if (seq < printk_log_first_seq) {
				/* messages are gone, move to next one */
				seq = printk_log_first_seq;
				idx = printk_log_first_idx;
				prev = 0;
			}
		}
	}

	if (clear) {
		printk_log_clear_seq = printk_log_next_seq;
		printk_log_clear_idx = printk_log_next_idx;
	}
	raw_spin_unlock_irq(&printk_logbuf_lock);

	kfree(text);
	return len;
}

int do_syslog(int type, char __user *buf, int len, bool from_file)
{
	bool clear = false;
	static int saved_console_loglevel = -1;
	int error;

	error = check_syslog_permissions(type, from_file);
	if (error)
		goto out;

	error = security_syslog(type);
	if (error)
		return error;

	switch (type) {
	case SYSLOG_ACTION_CLOSE:	/* Close log */
		break;
	case SYSLOG_ACTION_OPEN:	/* Open log */
		break;
	case SYSLOG_ACTION_READ:	/* Read from log */
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		error = wait_event_interruptible(printk_log_wait,
						 printk_syslog_seq != printk_log_next_seq);
		if (error)
			goto out;
		error = printk_syslog_print(buf, len);
		break;
	/* Read/clear last kernel messages */
	case SYSLOG_ACTION_READ_CLEAR:
		clear = true;
		/* FALL THRU */
	/* Read last kernel messages */
	case SYSLOG_ACTION_READ_ALL:
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		error = printk_syslog_print_all(buf, len, clear);
		break;
	/* Clear ring buffer */
	case SYSLOG_ACTION_CLEAR:
		printk_syslog_print_all(NULL, 0, true);
		break;
	/* Disable logging to console */
	case SYSLOG_ACTION_CONSOLE_OFF:
		if (saved_console_loglevel == -1)
			saved_console_loglevel = console_loglevel;
		console_loglevel = minimum_console_loglevel;
		break;
	/* Enable logging to console */
	case SYSLOG_ACTION_CONSOLE_ON:
		if (saved_console_loglevel != -1) {
			console_loglevel = saved_console_loglevel;
			saved_console_loglevel = -1;
		}
		break;
	/* Set level of messages printed to console */
	case SYSLOG_ACTION_CONSOLE_LEVEL:
		error = -EINVAL;
		if (len < 1 || len > 8)
			goto out;
		if (len < minimum_console_loglevel)
			len = minimum_console_loglevel;
		console_loglevel = len;
		/* Implicitly re-enable logging to console */
		saved_console_loglevel = -1;
		error = 0;
		break;
	/* Number of chars in the log buffer */
	case SYSLOG_ACTION_SIZE_UNREAD:
		raw_spin_lock_irq(&printk_logbuf_lock);
		if (printk_syslog_seq < printk_log_first_seq) {
			/* messages are gone, move to first one */
			printk_syslog_seq = printk_log_first_seq;
			printk_syslog_idx = printk_log_first_idx;
			printk_syslog_prev = 0;
			printk_syslog_partial = 0;
		}
		if (from_file) {
			/*
			 * Short-cut for poll(/"proc/kmsg") which simply checks
			 * for pending data, not the size; return the count of
			 * records, not the length.
			 */
			error = printk_log_next_idx - printk_syslog_idx;
		} else {
			u64 seq = printk_syslog_seq;
			u32 idx = printk_syslog_idx;
			enum printk_log_flags prev = printk_syslog_prev;

			error = 0;
			while (seq < printk_log_next_seq) {
				struct printk_log *msg = printk_log_from_idx(idx);

				error += printk_msg_print_text(msg, prev, true, NULL, 0);
				idx = printk_log_next(idx);
				seq++;
				prev = msg->flags;
			}
			error -= printk_syslog_partial;
		}
		raw_spin_unlock_irq(&printk_logbuf_lock);
		break;
	/* Size of the log buffer */
	case SYSLOG_ACTION_SIZE_BUFFER:
		error = printk_log_buf_len;
		break;
	default:
		error = -EINVAL;
		break;
	}
out:
	return error;
}

SYSCALL_DEFINE3(syslog, int, type, char __user *, buf, int, len)
{
	return do_syslog(type, buf, len, SYSLOG_FROM_CALL);
}

#endif
