#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/kexec.h>
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/kmsg_dump.h>
#include <linux/rculist.h>
#include <linux/stat.h>

#include "printk_log.h"

static DEFINE_SPINLOCK(dump_list_lock);
static LIST_HEAD(dump_list);

/**
 * kmsg_dump_register - register a kernel log dumper.
 * @dumper: pointer to the kmsg_dumper structure
 *
 * Adds a kernel log dumper to the system. The dump callback in the
 * structure will be called when the kernel oopses or panics and must be
 * set. Returns zero on success and %-EINVAL or %-EBUSY otherwise.
 */
int kmsg_dump_register(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EBUSY;

	/* The dump callback needs to be set */
	if (!dumper->dump)
		return -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	/* Don't allow registering multiple times */
	if (!dumper->registered) {
		dumper->registered = 1;
		list_add_tail_rcu(&dumper->list, &dump_list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_register);

/**
 * kmsg_dump_unregister - unregister a kmsg dumper.
 * @dumper: pointer to the kmsg_dumper structure
 *
 * Removes a dump device from the system. Returns zero on success and
 * %-EINVAL otherwise.
 */
int kmsg_dump_unregister(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	if (dumper->registered) {
		dumper->registered = 0;
		list_del_rcu(&dumper->list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);
	synchronize_rcu();

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_unregister);

static bool always_kmsg_dump;
module_param_named(always_kmsg_dump, always_kmsg_dump, bool, S_IRUGO | S_IWUSR);

/**
 * kmsg_dump - dump kernel log to kernel message dumpers.
 * @reason: the reason (oops, panic etc) for dumping
 *
 * Call each of the registered dumper's dump() callback, which can
 * retrieve the kmsg records with kmsg_dump_get_line() or
 * kmsg_dump_get_buffer().
 */
void kmsg_dump(enum kmsg_dump_reason reason)
{
	struct kmsg_dumper *dumper;
	unsigned long flags;

	if ((reason > KMSG_DUMP_OOPS) && !always_kmsg_dump)
		return;

	rcu_read_lock();
	list_for_each_entry_rcu(dumper, &dump_list, list) {
		if (dumper->max_reason && reason > dumper->max_reason)
			continue;

		/* initialize iterator with data about the stored records */
		dumper->active = true;

		raw_spin_lock_irqsave(&printk_logbuf_lock, flags);
		dumper->cur_seq = printk_log_clear_seq;
		dumper->cur_idx = printk_log_clear_idx;
		dumper->next_seq = printk_log_next_seq;
		dumper->next_idx = printk_log_next_idx;
		raw_spin_unlock_irqrestore(&printk_logbuf_lock, flags);

		/* invoke dumper which will iterate over records */
		dumper->dump(dumper, reason);

		/* reset iterator */
		dumper->active = false;
	}
	rcu_read_unlock();
}

/**
 * kmsg_dump_get_line_nolock - retrieve one kmsg log line (unlocked version)
 * @dumper: registered kmsg dumper
 * @syslog: include the "<4>" prefixes
 * @line: buffer to copy the line to
 * @size: maximum size of the buffer
 * @len: length of line placed into buffer
 *
 * Start at the beginning of the kmsg buffer, with the oldest kmsg
 * record, and copy one record into the provided buffer.
 *
 * Consecutive calls will return the next available record moving
 * towards the end of the buffer with the youngest messages.
 *
 * A return value of FALSE indicates that there are no more records to
 * read.
 *
 * The function is similar to kmsg_dump_get_line(), but grabs no locks.
 */
bool kmsg_dump_get_line_nolock(struct kmsg_dumper *dumper, bool syslog,
			       char *line, size_t size, size_t *len)
{
	struct printk_log *msg;
	size_t l = 0;
	bool ret = false;

	if (!dumper->active)
		goto out;

	if (dumper->cur_seq < printk_log_first_seq) {
		/* messages are gone, move to first available one */
		dumper->cur_seq = printk_log_first_seq;
		dumper->cur_idx = printk_log_first_idx;
	}

	/* last entry */
	if (dumper->cur_seq >= printk_log_next_seq)
		goto out;

	msg = printk_log_from_idx(dumper->cur_idx);
	l = printk_msg_print_text(msg, 0, syslog, line, size);

	dumper->cur_idx = printk_log_next(dumper->cur_idx);
	dumper->cur_seq++;
	ret = true;
out:
	if (len)
		*len = l;
	return ret;
}

/**
 * kmsg_dump_get_line - retrieve one kmsg log line
 * @dumper: registered kmsg dumper
 * @syslog: include the "<4>" prefixes
 * @line: buffer to copy the line to
 * @size: maximum size of the buffer
 * @len: length of line placed into buffer
 *
 * Start at the beginning of the kmsg buffer, with the oldest kmsg
 * record, and copy one record into the provided buffer.
 *
 * Consecutive calls will return the next available record moving
 * towards the end of the buffer with the youngest messages.
 *
 * A return value of FALSE indicates that there are no more records to
 * read.
 */
bool kmsg_dump_get_line(struct kmsg_dumper *dumper, bool syslog,
			char *line, size_t size, size_t *len)
{
	unsigned long flags;
	bool ret;

	raw_spin_lock_irqsave(&printk_logbuf_lock, flags);
	ret = kmsg_dump_get_line_nolock(dumper, syslog, line, size, len);
	raw_spin_unlock_irqrestore(&printk_logbuf_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(kmsg_dump_get_line);

/**
 * kmsg_dump_get_buffer - copy kmsg log lines
 * @dumper: registered kmsg dumper
 * @syslog: include the "<4>" prefixes
 * @buf: buffer to copy the line to
 * @size: maximum size of the buffer
 * @len: length of line placed into buffer
 *
 * Start at the end of the kmsg buffer and fill the provided buffer
 * with as many of the the *youngest* kmsg records that fit into it.
 * If the buffer is large enough, all available kmsg records will be
 * copied with a single call.
 *
 * Consecutive calls will fill the buffer with the next block of
 * available older records, not including the earlier retrieved ones.
 *
 * A return value of FALSE indicates that there are no more records to
 * read.
 */
bool kmsg_dump_get_buffer(struct kmsg_dumper *dumper, bool syslog,
			  char *buf, size_t size, size_t *len)
{
	unsigned long flags;
	u64 seq;
	u32 idx;
	u64 next_seq;
	u32 next_idx;
	enum printk_log_flags prev;
	size_t l = 0;
	bool ret = false;

	if (!dumper->active)
		goto out;

	raw_spin_lock_irqsave(&printk_logbuf_lock, flags);
	if (dumper->cur_seq < printk_log_first_seq) {
		/* messages are gone, move to first available one */
		dumper->cur_seq = printk_log_first_seq;
		dumper->cur_idx = printk_log_first_idx;
	}

	/* last entry */
	if (dumper->cur_seq >= dumper->next_seq) {
		raw_spin_unlock_irqrestore(&printk_logbuf_lock, flags);
		goto out;
	}

	/* calculate length of entire buffer */
	seq = dumper->cur_seq;
	idx = dumper->cur_idx;
	prev = 0;
	while (seq < dumper->next_seq) {
		struct printk_log *msg = printk_log_from_idx(idx);

		l += printk_msg_print_text(msg, prev, true, NULL, 0);
		idx = printk_log_next(idx);
		seq++;
		prev = msg->flags;
	}

	/* move first record forward until length fits into the buffer */
	seq = dumper->cur_seq;
	idx = dumper->cur_idx;
	prev = 0;
	while (l > size && seq < dumper->next_seq) {
		struct printk_log *msg = printk_log_from_idx(idx);

		l -= printk_msg_print_text(msg, prev, true, NULL, 0);
		idx = printk_log_next(idx);
		seq++;
		prev = msg->flags;
	}

	/* last message in next interation */
	next_seq = seq;
	next_idx = idx;

	l = 0;
	prev = 0;
	while (seq < dumper->next_seq) {
		struct printk_log *msg = printk_log_from_idx(idx);

		l += printk_msg_print_text(msg, prev, syslog, buf + l, size - l);
		idx = printk_log_next(idx);
		seq++;
		prev = msg->flags;
	}

	dumper->next_seq = next_seq;
	dumper->next_idx = next_idx;
	ret = true;
	raw_spin_unlock_irqrestore(&printk_logbuf_lock, flags);
out:
	if (len)
		*len = l;
	return ret;
}
EXPORT_SYMBOL_GPL(kmsg_dump_get_buffer);

/**
 * kmsg_dump_rewind_nolock - reset the interator (unlocked version)
 * @dumper: registered kmsg dumper
 *
 * Reset the dumper's iterator so that kmsg_dump_get_line() and
 * kmsg_dump_get_buffer() can be called again and used multiple
 * times within the same dumper.dump() callback.
 *
 * The function is similar to kmsg_dump_rewind(), but grabs no locks.
 */
void kmsg_dump_rewind_nolock(struct kmsg_dumper *dumper)
{
	dumper->cur_seq = printk_log_clear_seq;
	dumper->cur_idx = printk_log_clear_idx;
	dumper->next_seq = printk_log_next_seq;
	dumper->next_idx = printk_log_next_idx;
}

/**
 * kmsg_dump_rewind - reset the interator
 * @dumper: registered kmsg dumper
 *
 * Reset the dumper's iterator so that kmsg_dump_get_line() and
 * kmsg_dump_get_buffer() can be called again and used multiple
 * times within the same dumper.dump() callback.
 */
void kmsg_dump_rewind(struct kmsg_dumper *dumper)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&printk_logbuf_lock, flags);
	kmsg_dump_rewind_nolock(dumper);
	raw_spin_unlock_irqrestore(&printk_logbuf_lock, flags);
}
EXPORT_SYMBOL_GPL(kmsg_dump_rewind);
