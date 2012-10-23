/*
 *  linux/fs/hfsplus/ioctl.c
 *
 * Copyright (C) 2003
 * Ethan Benson <erbenson@alaska.net>
 * partially derived from linux/fs/ext2/ioctl.c
 * Copyright (C) 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 *
 * hfsplus ioctls
 */

#include <linux/capability.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/sched.h>
#include <linux/xattr.h>
#include <asm/uaccess.h>
#include "hfsplus_fs.h"

/*
 * "Blessing" an HFS+ filesystem writes metadata to the superblock informing
 * the platform firmware which file to boot from
 */
static int hfsplus_ioctl_bless(struct file *file, int __user *user_flags)
{
	struct dentry *dentry = file->f_path.dentry;
	struct inode *inode = dentry->d_inode;
	struct hfsplus_sb_info *sbi = HFSPLUS_SB(inode->i_sb);
	struct hfsplus_vh *vh = sbi->s_vhdr;
	struct hfsplus_vh *bvh = sbi->s_backup_vhdr;
	u32 cnid = (unsigned long)dentry->d_fsdata;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	mutex_lock(&sbi->vh_mutex);

	/* Directory containing the bootable system */
	vh->finder_info[0] = bvh->finder_info[0] =
		cpu_to_be32(parent_ino(dentry));

	/*
	 * Bootloader. Just using the inode here breaks in the case of
	 * hard links - the firmware wants the ID of the hard link file,
	 * but the inode points at the indirect inode
	 */
	vh->finder_info[1] = bvh->finder_info[1] = cpu_to_be32(cnid);

	/* Per spec, the OS X system folder - same as finder_info[0] here */
	vh->finder_info[5] = bvh->finder_info[5] =
		cpu_to_be32(parent_ino(dentry));

	mutex_unlock(&sbi->vh_mutex);
	return 0;
}

static int hfsplus_ioctl_getflags(struct file *file, int __user *user_flags)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct hfsplus_inode_info *hip = HFSPLUS_I(inode);
	unsigned int flags = 0;

	if (inode->i_flags & S_IMMUTABLE)
		flags |= FS_IMMUTABLE_FL;
	if (inode->i_flags & S_APPEND)
		flags |= FS_APPEND_FL;
	if (hip->userflags & HFSPLUS_FLG_NODUMP)
		flags |= FS_NODUMP_FL;

	return put_user(flags, user_flags);
}

static int hfsplus_ioctl_setflags(struct file *file, int __user *user_flags)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct hfsplus_inode_info *hip = HFSPLUS_I(inode);
	unsigned int flags;
	int err = 0;

	err = mnt_want_write_file(file);
	if (err)
		goto out;

	if (!inode_owner_or_capable(inode)) {
		err = -EACCES;
		goto out_drop_write;
	}

	if (get_user(flags, user_flags)) {
		err = -EFAULT;
		goto out_drop_write;
	}

	mutex_lock(&inode->i_mutex);

	if ((flags & (FS_IMMUTABLE_FL|FS_APPEND_FL)) ||
	    inode->i_flags & (S_IMMUTABLE|S_APPEND)) {
		if (!capable(CAP_LINUX_IMMUTABLE)) {
			err = -EPERM;
			goto out_unlock_inode;
		}
	}

	/* don't silently ignore unsupported ext2 flags */
	if (flags & ~(FS_IMMUTABLE_FL|FS_APPEND_FL|FS_NODUMP_FL)) {
		err = -EOPNOTSUPP;
		goto out_unlock_inode;
	}

	if (flags & FS_IMMUTABLE_FL)
		inode->i_flags |= S_IMMUTABLE;
	else
		inode->i_flags &= ~S_IMMUTABLE;

	if (flags & FS_APPEND_FL)
		inode->i_flags |= S_APPEND;
	else
		inode->i_flags &= ~S_APPEND;

	if (flags & FS_NODUMP_FL)
		hip->userflags |= HFSPLUS_FLG_NODUMP;
	else
		hip->userflags &= ~HFSPLUS_FLG_NODUMP;

	inode->i_ctime = CURRENT_TIME_SEC;
	mark_inode_dirty(inode);

out_unlock_inode:
	mutex_unlock(&inode->i_mutex);
out_drop_write:
	mnt_drop_write_file(file);
out:
	return err;
}

long hfsplus_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HFSPLUS_IOC_EXT2_GETFLAGS:
		return hfsplus_ioctl_getflags(file, argp);
	case HFSPLUS_IOC_EXT2_SETFLAGS:
		return hfsplus_ioctl_setflags(file, argp);
	case HFSPLUS_IOC_BLESS:
		return hfsplus_ioctl_bless(file, argp);
	default:
		return -ENOTTY;
	}
}

static int strcmp_xattr_finder_info(const char *name)
{
	if (name) {
		return strncmp(name, HFSPLUS_XATTR_FINDER_INFO_NAME,
				sizeof(HFSPLUS_XATTR_FINDER_INFO_NAME));
	}
	return -1;
}

static int strcmp_xattr_acl(const char *name)
{
	if (name) {
		return strncmp(name, HFSPLUS_XATTR_ACL_NAME,
				sizeof(HFSPLUS_XATTR_ACL_NAME));
	}
	return -1;
}

int hfsplus_setxattr(struct dentry *dentry, const char *name,
			const void *value, size_t size, int flags)
{
	int err = 0;
	struct inode *inode = dentry->d_inode;
	struct hfs_find_data cat_fd;
	hfsplus_cat_entry entry;
	u16 cat_entry_flags, cat_entry_type;
	u16 folder_finderinfo_len = sizeof(struct DInfo) +
					sizeof(struct DXInfo);
	u16 file_finderinfo_len = sizeof(struct FInfo) +
					sizeof(struct FXInfo);

	if ((!S_ISREG(inode->i_mode) &&
			!S_ISDIR(inode->i_mode)) ||
				HFSPLUS_IS_RSRC(inode))
		return -EOPNOTSUPP;

	err = hfs_find_init(HFSPLUS_SB(inode->i_sb)->cat_tree, &cat_fd);
	if (err) {
		printk(KERN_ERR "hfs: can't init xattr find struct\n");
		return err;
	}

	err = hfsplus_find_cat(inode->i_sb, inode->i_ino, &cat_fd);
	if (err) {
		printk(KERN_ERR "hfs: catalog searching failed\n");
		goto end_setxattr;
	}

	if (!strcmp_xattr_finder_info(name)) {
		if (flags & XATTR_CREATE) {
			printk(KERN_ERR "hfs: xattr exists yet\n");
			err = -EOPNOTSUPP;
			goto end_setxattr;
		}
		hfs_bnode_read(cat_fd.bnode, &entry, cat_fd.entryoffset,
					sizeof(hfsplus_cat_entry));
		if (be16_to_cpu(entry.type) == HFSPLUS_FOLDER) {
			if (size == folder_finderinfo_len) {
				memcpy(&entry.folder.user_info, value,
						folder_finderinfo_len);
				hfs_bnode_write(cat_fd.bnode, &entry,
					cat_fd.entryoffset,
					sizeof(struct hfsplus_cat_folder));
				hfsplus_mark_inode_dirty(inode,
						HFSPLUS_I_CAT_DIRTY);
			} else {
				err = -ERANGE;
				goto end_setxattr;
			}
		} else if (be16_to_cpu(entry.type) == HFSPLUS_FILE) {
			if (size == file_finderinfo_len) {
				memcpy(&entry.file.user_info, value,
						file_finderinfo_len);
				hfs_bnode_write(cat_fd.bnode, &entry,
					cat_fd.entryoffset,
					sizeof(struct hfsplus_cat_file));
				hfsplus_mark_inode_dirty(inode,
						HFSPLUS_I_CAT_DIRTY);
			} else {
				err = -ERANGE;
				goto end_setxattr;
			}
		} else {
			err = -EOPNOTSUPP;
			goto end_setxattr;
		}
		goto end_setxattr;
	}

	if (!HFSPLUS_SB(inode->i_sb)->attr_tree) {
		err = -EOPNOTSUPP;
		goto end_setxattr;
	}

	if (hfsplus_attr_exists(inode, name)) {
		if (flags & XATTR_CREATE) {
			printk(KERN_ERR "hfs: xattr exists yet\n");
			err = -EOPNOTSUPP;
			goto end_setxattr;
		}
		err = hfsplus_delete_attr(inode, name);
		if (err)
			goto end_setxattr;
		err = hfsplus_create_attr(inode, name, value, size);
		if (err)
			goto end_setxattr;
	} else {
		if (flags & XATTR_REPLACE) {
			printk(KERN_ERR "hfs: cannot replace xattr\n");
			err = -EOPNOTSUPP;
			goto end_setxattr;
		}
		err = hfsplus_create_attr(inode, name, value, size);
		if (err)
			goto end_setxattr;
	}

	cat_entry_type = hfs_bnode_read_u16(cat_fd.bnode, cat_fd.entryoffset);
	if (cat_entry_type == HFSPLUS_FOLDER) {
		cat_entry_flags = hfs_bnode_read_u16(cat_fd.bnode,
				    cat_fd.entryoffset +
				    offsetof(struct hfsplus_cat_folder, flags));
		cat_entry_flags |= HFSPLUS_XATTR_EXISTS;
		if (!strcmp_xattr_acl(name))
			cat_entry_flags |= HFSPLUS_ACL_EXISTS;
		hfs_bnode_write_u16(cat_fd.bnode, cat_fd.entryoffset +
				offsetof(struct hfsplus_cat_folder, flags),
				cat_entry_flags);
		hfsplus_mark_inode_dirty(inode, HFSPLUS_I_CAT_DIRTY);
	} else if (cat_entry_type == HFSPLUS_FILE) {
		cat_entry_flags = hfs_bnode_read_u16(cat_fd.bnode,
				    cat_fd.entryoffset +
				    offsetof(struct hfsplus_cat_file, flags));
		cat_entry_flags |= HFSPLUS_XATTR_EXISTS;
		if (!strcmp_xattr_acl(name))
			cat_entry_flags |= HFSPLUS_ACL_EXISTS;
		hfs_bnode_write_u16(cat_fd.bnode, cat_fd.entryoffset +
				    offsetof(struct hfsplus_cat_file, flags),
				    cat_entry_flags);
		hfsplus_mark_inode_dirty(inode, HFSPLUS_I_CAT_DIRTY);
	} else {
		printk(KERN_ERR "hfs: invalid catalog entry type\n");
		err = -EIO;
		goto end_setxattr;
	}

end_setxattr:
	hfs_find_exit(&cat_fd);
	return err;
}

static ssize_t hfsplus_getxattr_finder_info(struct dentry *dentry,
						void *value, size_t size)
{
	ssize_t res = 0;
	struct inode *inode = dentry->d_inode;
	struct hfs_find_data fd;
	u16 entry_type;
	u16 folder_rec_len = sizeof(struct DInfo) + sizeof(struct DXInfo);
	u16 file_rec_len = sizeof(struct FInfo) + sizeof(struct FXInfo);
	u16 record_len = max(folder_rec_len, file_rec_len);
	u8 folder_finder_info[sizeof(struct DInfo) + sizeof(struct DXInfo)];
	u8 file_finder_info[sizeof(struct FInfo) + sizeof(struct FXInfo)];

	if (size >= record_len) {
		res = hfs_find_init(HFSPLUS_SB(inode->i_sb)->cat_tree, &fd);
		if (res) {
			printk(KERN_ERR "hfs: can't init xattr find struct\n");
			return res;
		}
		res = hfsplus_find_cat(inode->i_sb, inode->i_ino, &fd);
		if (res)
			goto end_getxattr_finder_info;
		entry_type = hfs_bnode_read_u16(fd.bnode, fd.entryoffset);

		if (entry_type == HFSPLUS_FOLDER) {
			hfs_bnode_read(fd.bnode, folder_finder_info,
				fd.entryoffset +
				offsetof(struct hfsplus_cat_folder, user_info),
				folder_rec_len);
			memcpy(value, folder_finder_info, folder_rec_len);
			res = folder_rec_len;
		} else if (entry_type == HFSPLUS_FILE) {
			hfs_bnode_read(fd.bnode, file_finder_info,
				fd.entryoffset +
				offsetof(struct hfsplus_cat_file, user_info),
				file_rec_len);
			memcpy(value, file_finder_info, file_rec_len);
			res = file_rec_len;
		} else {
			res = -EOPNOTSUPP;
			goto end_getxattr_finder_info;
		}
	} else
		res = size ? -ERANGE : record_len;

end_getxattr_finder_info:
	if (size >= record_len)
		hfs_find_exit(&fd);
	return res;
}

ssize_t hfsplus_getxattr(struct dentry *dentry, const char *name,
			 void *value, size_t size)
{
	struct inode *inode = dentry->d_inode;
	struct hfs_find_data fd;
	hfsplus_attr_entry *entry;
	u32 record_type;
	u16 record_length = 0;
	ssize_t res = 0;

	if ((!S_ISREG(inode->i_mode) &&
			!S_ISDIR(inode->i_mode)) ||
				HFSPLUS_IS_RSRC(inode))
		return -EOPNOTSUPP;

	if (!strcmp_xattr_finder_info(name))
		return hfsplus_getxattr_finder_info(dentry, value, size);

	if (!HFSPLUS_SB(inode->i_sb)->attr_tree)
		return -EOPNOTSUPP;

	entry = hfsplus_alloc_attr_entry();
	if (!entry) {
		printk(KERN_ERR "hfs: can't allocate xattr entry\n");
		return -ENOMEM;
	}

	res = hfs_find_init(HFSPLUS_SB(inode->i_sb)->attr_tree, &fd);
	if (res) {
		printk(KERN_ERR "hfs: can't init xattr find struct\n");
		goto failed_getxattr_init;
	}

	res = hfsplus_find_attr(inode->i_sb, inode->i_ino, name, &fd);
	if (res) {
		if (res == -ENOENT)
			res = -ENODATA;
		else
			printk(KERN_ERR "hfs: xattr searching failed\n");
		goto out;
	}

	hfs_bnode_read(fd.bnode, &record_type,
			fd.entryoffset, sizeof(record_type));
	record_type = be32_to_cpu(record_type);
	if (record_type == HFSPLUS_ATTR_INLINE_DATA) {
		record_length = hfs_bnode_read_u16(fd.bnode,
				fd.entryoffset +
				offsetof(struct hfsplus_attr_inline_data,
				length));
		if (record_length > HFSPLUS_MAX_INLINE_DATA_SIZE) {
			printk(KERN_ERR "hfs: invalid xattr record size\n");
			res = -EIO;
			goto out;
		}
	} else if (record_type == HFSPLUS_ATTR_FORK_DATA ||
			record_type == HFSPLUS_ATTR_EXTENTS) {
		printk(KERN_ERR "hfs: only inline data xattr are supported\n");
		res = -EOPNOTSUPP;
		goto out;
	} else {
		printk(KERN_ERR "hfs: invalid xattr record\n");
		res = -EIO;
		goto out;
	}

	if (size) {
		hfs_bnode_read(fd.bnode, entry, fd.entryoffset,
				offsetof(struct hfsplus_attr_inline_data,
					raw_bytes) + record_length);
	}

	if (size >= record_length) {
		memcpy(value, entry->inline_data.raw_bytes, record_length);
		res = record_length;
	} else
		res = size ? -ERANGE : record_length;

out:
	hfs_find_exit(&fd);

failed_getxattr_init:
	hfsplus_destroy_attr_entry(entry);
	return res;
}

static ssize_t hfsplus_listxattr_finder_info(struct dentry *dentry,
						char *buffer, size_t size)
{
	ssize_t res = 0;
	struct inode *inode = dentry->d_inode;
	struct hfs_find_data fd;
	u16 entry_type;
	u8 folder_finder_info[sizeof(struct DInfo) + sizeof(struct DXInfo)];
	u8 file_finder_info[sizeof(struct FInfo) + sizeof(struct FXInfo)];
	unsigned long len, found_bit;

	res = hfs_find_init(HFSPLUS_SB(inode->i_sb)->cat_tree, &fd);
	if (res) {
		printk(KERN_ERR "hfs: can't init xattr find struct\n");
		return res;
	}

	res = hfsplus_find_cat(inode->i_sb, inode->i_ino, &fd);
	if (res)
		goto end_listxattr_finder_info;

	entry_type = hfs_bnode_read_u16(fd.bnode, fd.entryoffset);
	if (entry_type == HFSPLUS_FOLDER) {
		len = sizeof(struct DInfo) + sizeof(struct DXInfo);
		hfs_bnode_read(fd.bnode, folder_finder_info,
				fd.entryoffset +
				offsetof(struct hfsplus_cat_folder, user_info),
				len);
		found_bit = find_first_bit((void *)folder_finder_info, len*8);
	} else if (entry_type == HFSPLUS_FILE) {
		len = sizeof(struct FInfo) + sizeof(struct FXInfo);
		hfs_bnode_read(fd.bnode, file_finder_info,
				fd.entryoffset +
				offsetof(struct hfsplus_cat_file, user_info),
				len);
		found_bit = find_first_bit((void *)file_finder_info, len*8);
	} else {
		res = -EOPNOTSUPP;
		goto end_listxattr_finder_info;
	}

	if (found_bit >= (len*8))
		res = 0;
	else {
		if (!buffer || !size) {
			res = sizeof(HFSPLUS_XATTR_FINDER_INFO_NAME);
		} else if (size < sizeof(HFSPLUS_XATTR_FINDER_INFO_NAME)) {
			res = -ERANGE;
		} else {
			strncpy(buffer, HFSPLUS_XATTR_FINDER_INFO_NAME,
				    sizeof(HFSPLUS_XATTR_FINDER_INFO_NAME));
			res = sizeof(HFSPLUS_XATTR_FINDER_INFO_NAME);
		}
	}

end_listxattr_finder_info:
	hfs_find_exit(&fd);

	return res;
}

ssize_t hfsplus_listxattr(struct dentry *dentry, char *buffer, size_t size)
{
	ssize_t err;
	ssize_t res = 0;
	struct inode *inode = dentry->d_inode;
	struct hfs_find_data fd;
	u16 key_len = 0;
	struct hfsplus_attr_key attr_key;
	char strbuf[HFSPLUS_ATTR_MAX_STRLEN + 1] = {0};
	int name_len;

	if ((!S_ISREG(inode->i_mode) &&
			!S_ISDIR(inode->i_mode)) ||
				HFSPLUS_IS_RSRC(inode))
		return -EOPNOTSUPP;

	res = hfsplus_listxattr_finder_info(dentry, buffer, size);
	if (res < 0)
		return res;
	else if (!HFSPLUS_SB(inode->i_sb)->attr_tree)
		return (res == 0) ? -EOPNOTSUPP : res;

	err = hfs_find_init(HFSPLUS_SB(inode->i_sb)->attr_tree, &fd);
	if (err) {
		printk(KERN_ERR "hfs: can't init xattr find struct\n");
		return err;
	}

	err = hfsplus_find_attr(inode->i_sb, inode->i_ino, NULL, &fd);
	if (err) {
		if (err == -ENOENT) {
			if (res == 0)
				res = -ENODATA;
			goto end_listxattr;
		} else {
			res = err;
			goto end_listxattr;
		}
	}

	for (;;) {
		key_len = hfs_bnode_read_u16(fd.bnode, fd.keyoffset);
		if (key_len == 0 || key_len > fd.tree->max_key_len) {
			printk(KERN_ERR "hfs: invalid xattr key length: %d\n",
							key_len);
			res = -EIO;
			goto end_listxattr;
		}

		hfs_bnode_read(fd.bnode, &attr_key,
				fd.keyoffset, key_len + sizeof(key_len));

		if (be32_to_cpu(attr_key.cnid) != inode->i_ino)
			goto end_listxattr;

		name_len = HFSPLUS_ATTR_MAX_STRLEN;
		if (hfsplus_uni2asc(inode->i_sb,
			(const struct hfsplus_unistr *)&fd.key->attr.key_name,
					strbuf, &name_len)) {
			printk(KERN_ERR "hfs: unicode conversion failed\n");
			res = -EIO;
			goto end_listxattr;
		}

		if (!buffer || !size)
			res += name_len + 1;
		else if (size < (res + name_len + 1)) {
			res = -ERANGE;
			goto end_listxattr;
		} else {
			strncpy(buffer + res, strbuf, name_len);
			memset(buffer + res + name_len, 0, 1);
			res += name_len + 1;
		}

		if (hfs_brec_goto(&fd, 1))
			goto end_listxattr;
	}

end_listxattr:
	hfs_find_exit(&fd);
	return res;
}

int hfsplus_removexattr(struct dentry *dentry, const char *name)
{
	int err = 0;
	struct inode *inode = dentry->d_inode;
	struct hfs_find_data cat_fd;
	u16 flags;
	u16 cat_entry_type;
	int is_xattr_acl_deleted = 0;
	int is_all_xattrs_deleted = 0;

	if ((!S_ISREG(inode->i_mode) &&
			!S_ISDIR(inode->i_mode)) ||
				HFSPLUS_IS_RSRC(inode))
		return -EOPNOTSUPP;

	if (!HFSPLUS_SB(inode->i_sb)->attr_tree)
		return -EOPNOTSUPP;

	if (!strcmp_xattr_finder_info(name))
		return -EOPNOTSUPP;

	err = hfs_find_init(HFSPLUS_SB(inode->i_sb)->cat_tree, &cat_fd);
	if (err) {
		printk(KERN_ERR "hfs: can't init xattr find struct\n");
		return err;
	}

	err = hfsplus_find_cat(inode->i_sb, inode->i_ino, &cat_fd);
	if (err) {
		printk(KERN_ERR "hfs: catalog searching failed\n");
		goto end_removexattr;
	}

	err = hfsplus_delete_attr(inode, name);
	if (err)
		goto end_removexattr;

	is_xattr_acl_deleted = !strcmp_xattr_acl(name);
	is_all_xattrs_deleted = !hfsplus_attr_exists(inode, NULL);

	if (!is_xattr_acl_deleted && !is_all_xattrs_deleted)
		goto end_removexattr;

	cat_entry_type = hfs_bnode_read_u16(cat_fd.bnode, cat_fd.entryoffset);

	if (cat_entry_type == HFSPLUS_FOLDER) {
		flags = hfs_bnode_read_u16(cat_fd.bnode, cat_fd.entryoffset +
				offsetof(struct hfsplus_cat_folder, flags));
		if (is_xattr_acl_deleted)
			flags &= ~HFSPLUS_ACL_EXISTS;
		if (is_all_xattrs_deleted)
			flags &= ~HFSPLUS_XATTR_EXISTS;
		hfs_bnode_write_u16(cat_fd.bnode, cat_fd.entryoffset +
				offsetof(struct hfsplus_cat_folder, flags),
				flags);
		hfsplus_mark_inode_dirty(inode, HFSPLUS_I_CAT_DIRTY);
	} else if (cat_entry_type == HFSPLUS_FILE) {
		flags = hfs_bnode_read_u16(cat_fd.bnode, cat_fd.entryoffset +
				offsetof(struct hfsplus_cat_file, flags));
		if (is_xattr_acl_deleted)
			flags &= ~HFSPLUS_ACL_EXISTS;
		if (is_all_xattrs_deleted)
			flags &= ~HFSPLUS_XATTR_EXISTS;
		hfs_bnode_write_u16(cat_fd.bnode, cat_fd.entryoffset +
				offsetof(struct hfsplus_cat_file, flags),
				flags);
		hfsplus_mark_inode_dirty(inode, HFSPLUS_I_CAT_DIRTY);
	} else {
		printk(KERN_ERR "hfs: invalid catalog entry type\n");
		err = -EIO;
		goto end_removexattr;
	}

end_removexattr:
	hfs_find_exit(&cat_fd);
	return err;
}
