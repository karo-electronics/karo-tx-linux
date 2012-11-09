/*
 * include/linux/balloon_compaction.h
 *
 * Common interface definitions for making balloon pages movable to compaction.
 *
 * Copyright (C) 2012, Red Hat, Inc.  Rafael Aquini <aquini@redhat.com>
 */
#ifndef _LINUX_BALLOON_COMPACTION_H
#define _LINUX_BALLOON_COMPACTION_H
#include <linux/pagemap.h>
#include <linux/migrate.h>
#include <linux/gfp.h>
#include <linux/err.h>

/*
 * Balloon device information descriptor.
 * This struct is used to allow the common balloon compaction interface
 * procedures to find the proper balloon device holding memory pages they'll
 * have to cope for page compaction / migration, as well as it serves the
 * balloon driver as a page book-keeper for its registered balloon devices.
 */
struct balloon_dev_info {
	void *balloon_device;		/* balloon device descriptor */
	struct address_space *mapping;	/* balloon special page->mapping */
	unsigned long isolated_pages;	/* # of isolated pages for migration */
	spinlock_t pages_lock;		/* Protection to pages list */
	struct list_head pages;		/* Pages enqueued & handled to Host */
};

extern struct page *balloon_page_enqueue(struct balloon_dev_info *b_dev_info);
extern struct page *balloon_page_dequeue(struct balloon_dev_info *b_dev_info);
extern struct balloon_dev_info *balloon_devinfo_alloc(
						void *balloon_dev_descriptor);

static inline void balloon_devinfo_free(struct balloon_dev_info *b_dev_info)
{
	kfree(b_dev_info);
}

#ifdef CONFIG_BALLOON_COMPACTION
extern bool balloon_page_isolate(struct page *page);
extern void balloon_page_putback(struct page *page);
extern int balloon_page_migrate(struct page *newpage,
				struct page *page, enum migrate_mode mode);
extern struct address_space
*balloon_mapping_alloc(struct balloon_dev_info *b_dev_info,
			const struct address_space_operations *a_ops);

static inline void balloon_mapping_free(struct address_space *balloon_mapping)
{
	kfree(balloon_mapping);
}

/*
 * balloon_page_insert - insert a page into the balloon's page list and make
 *		         the page->mapping assignment accordingly.
 * @page    : page to be assigned as a 'balloon page'
 * @mapping : allocated special 'balloon_mapping'
 * @head    : balloon's device page list head
 */
static inline void balloon_page_insert(struct page *page,
				       struct address_space *mapping,
				       struct list_head *head)
{
	list_add(&page->lru, head);
	/*
	 * Make sure the page is already inserted on balloon's page list
	 * before assigning its ->mapping.
	 */
	smp_wmb();
	page->mapping = mapping;
}

/*
 * balloon_page_delete - clear the page->mapping and delete the page from
 *			 balloon's page list accordingly.
 * @page    : page to be released from balloon's page list
 */
static inline void balloon_page_delete(struct page *page)
{
	page->mapping = NULL;
	/*
	 * Make sure page->mapping is cleared before we proceed with
	 * balloon's page list deletion.
	 */
	smp_wmb();
	list_del(&page->lru);
}

/*
 * __is_movable_balloon_page - helper to perform @page mapping->flags tests
 */
static inline bool __is_movable_balloon_page(struct page *page)
{
	/*
	 * we might attempt to read ->mapping concurrently to other
	 * threads trying to write to it.
	 */
	struct address_space *mapping = ACCESS_ONCE(page->mapping);
	smp_read_barrier_depends();
	return mapping_balloon(mapping);
}

/*
 * balloon_page_movable - test page->mapping->flags to identify balloon pages
 *			  that can be moved by compaction/migration.
 *
 * This function is used at core compaction's page isolation scheme, therefore
 * most pages exposed to it are not enlisted as balloon pages and so, to avoid
 * undesired side effects like racing against __free_pages(), we cannot afford
 * holding the page locked while testing page->mapping->flags here.
 *
 * As we might return false positives in the case of a balloon page being just
 * released under us, the page->mapping->flags need to be re-tested later,
 * under the proper page lock, at the functions that will be coping with the
 * balloon page case.
 */
static inline bool balloon_page_movable(struct page *page)
{
	/*
	 * Before dereferencing and testing mapping->flags, lets make sure
	 * this is not a page that uses ->mapping in a different way
	 */
	if (!PageSlab(page) && !PageSwapCache(page) && !PageAnon(page) &&
	    !page_mapped(page))
		return __is_movable_balloon_page(page);

	return false;
}

/*
 * balloon_page_device - get the b_dev_info descriptor for the balloon device
 *			 that enqueues the given page.
 */
static inline struct balloon_dev_info *balloon_page_device(struct page *page)
{
	struct address_space *mapping = page->mapping;
	if (likely(mapping))
		return mapping->private_data;

	return NULL;
}

static inline gfp_t balloon_mapping_gfp_mask(void)
{
	return GFP_HIGHUSER_MOVABLE;
}

static inline void balloon_event_count(enum vm_event_item item)
{
	count_vm_event(item);
}

static inline bool balloon_compaction_check(void)
{
	return true;
}

#else /* !CONFIG_BALLOON_COMPACTION */

static inline void *balloon_mapping_alloc(void *balloon_device,
				const struct address_space_operations *a_ops)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static inline void balloon_mapping_free(struct address_space *balloon_mapping)
{
	return;
}

static inline void balloon_page_insert(struct page *page,
				       struct address_space *mapping,
				       struct list_head *head)
{
	list_add(&page->lru, head);
}

static inline void balloon_page_delete(struct page *page)
{
	list_del(&page->lru);
}

static inline bool balloon_page_movable(struct page *page)
{
	return false;
}

static inline bool balloon_page_isolate(struct page *page)
{
	return false;
}

static inline void balloon_page_putback(struct page *page)
{
	return;
}

static inline int balloon_page_migrate(struct page *newpage,
				struct page *page, enum migrate_mode mode)
{
	return 0;
}

static inline gfp_t balloon_mapping_gfp_mask(void)
{
	return GFP_HIGHUSER;
}

/* A macro, to avoid generating references to the undefined COMPACTBALLOON* */
#define balloon_event_count(item) do { } while (0)

static inline bool balloon_compaction_check(void)
{
	return false;
}
#endif /* CONFIG_BALLOON_COMPACTION */
#endif /* _LINUX_BALLOON_COMPACTION_H */
