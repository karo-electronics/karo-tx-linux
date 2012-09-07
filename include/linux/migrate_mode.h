#ifndef MIGRATE_MODE_H_INCLUDED
#define MIGRATE_MODE_H_INCLUDED

/* MIGRATE_ASYNC means never block */
#define MIGRATE_ASYNC		((__force migrate_mode_t)0x1)
/*
 * MIGRATE_SYNC_LIGHT in the current implementation means to allow blocking
 *	on most operations but not ->writepage as the potential stall time
 *	is too significant
 */
#define MIGRATE_SYNC_LIGHT	((__force migrate_mode_t)0x2)
/*
 * MIGRATE_SYNC will block when migrating pages
 */
#define MIGRATE_SYNC		((__force migrate_mode_t)0x4)

/*
 * MIGRTATE_DISCARD will discard clean cache page instead of migration.
 * MIGRATE_ASYNC, MIGRATE_SYNC_LIGHT, MIGRATE_SYNC shouldn't be used
 * together with OR flag in current implementation.
 */
#define MIGRATE_DISCARD		((__force migrate_mode_t)0x8)

typedef unsigned __bitwise__ migrate_mode_t;

#endif		/* MIGRATE_MODE_H_INCLUDED */
