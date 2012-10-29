/* fs/fat/nfs.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/exportfs.h>
#include "fat.h"

struct fat_fid {
	u32 ino;
	u32 gen;
	u64 i_pos;
	u32 parent_ino;
	u32 parent_gen;
} __packed;

/**
 * Look up a directory inode given its starting cluster.
 */
static struct inode *fat_dget(struct super_block *sb, int i_logstart)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct hlist_head *head;
	struct hlist_node *_p;
	struct msdos_inode_info *i;
	struct inode *inode = NULL;

	head = sbi->dir_hashtable + fat_dir_hash(i_logstart);
	spin_lock(&sbi->dir_hash_lock);
	hlist_for_each_entry(i, _p, head, i_dir_hash) {
		BUG_ON(i->vfs_inode.i_sb != sb);
		if (i->i_logstart != i_logstart)
			continue;
		inode = igrab(&i->vfs_inode);
		if (inode)
			break;
	}
	spin_unlock(&sbi->dir_hash_lock);
	return inode;
}

static struct inode *fat_nfs_get_inode(struct super_block *sb,
				       u64 ino, u32 generation, loff_t i_pos)
{
	struct inode *inode;

	if ((ino < MSDOS_ROOT_INO) || (ino == MSDOS_FSINFO_INO))
		return NULL;

	inode = ilookup(sb, ino);
	if (inode && generation && (inode->i_generation != generation)) {
		iput(inode);
		inode = NULL;
	}
	if (inode == NULL && MSDOS_SB(sb)->options.nfs == FAT_NFS_NOSTALE_RO) {
		struct buffer_head *bh = NULL;
		struct msdos_dir_entry *de ;
		sector_t blocknr;
		int offset;
		fat_get_blknr_offset(MSDOS_SB(sb), i_pos, &blocknr, &offset);
		bh = sb_bread(sb, blocknr);
		if (!bh) {
			fat_msg(sb, KERN_ERR,
				"unable to read block(%llu) for building NFS inode",
				(llu)blocknr);
			return inode;
		}
		de = (struct msdos_dir_entry *)bh->b_data;
		/* If a file is deleted on server and client is not updated
		 * yet, we must not build the inode upon a lookup call.
		 */
		if (IS_FREE(de[offset].name))
			inode = NULL;
		else
			inode = fat_build_inode(sb, &de[offset], i_pos);
		brelse(bh);
	}

	return inode;
}


int
fat_encode_fh(struct inode *inode, __u32 *fh, int *lenp, struct inode *parent)
{
	int len = *lenp;
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);
	struct fat_fid *fid = (struct fat_fid *) fh;
	loff_t i_pos;
	int type = FILEID_INO32_GEN;

	if (parent && (len < 5)) {
		*lenp = 5;
		return 255;
	} else if (len < 3) {
		*lenp = 3;
		return 255;
	}

	i_pos = fat_i_pos_read(sbi, inode);
	*lenp = 3;
	fid->ino = inode->i_ino;
	fid->gen = inode->i_generation;
	fid->i_pos = i_pos;
	if (parent) {
		fid->parent_ino = parent->i_ino;
		fid->parent_gen = parent->i_generation;
		type = FILEID_INO32_GEN_PARENT;
		*lenp = 5;
	}

	return type;
}

/**
 * Map a NFS file handle to a corresponding dentry.
 * The dentry may or may not be connected to the filesystem root.
 */
struct dentry *fat_fh_to_dentry(struct super_block *sb, struct fid *fh,
				int fh_len, int fh_type)
{
	struct inode *inode = NULL;
	struct fat_fid *fid = (struct fat_fid *)fh;
	if (fh_len < 3)
		return NULL;

	switch (fh_type) {
	case FILEID_INO32_GEN:
	case FILEID_INO32_GEN_PARENT:
		inode = fat_nfs_get_inode(sb, fid->ino, fid->gen, fid->i_pos);

		break;
	}

	return d_obtain_alias(inode);
}

/*
 * Find the parent for a file specified by NFS handle.
 * This requires that the handle contain the i_ino of the parent.
 */
struct dentry *fat_fh_to_parent(struct super_block *sb, struct fid *fh,
				int fh_len, int fh_type)
{
	struct inode *inode = NULL;
	struct fat_fid *fid = (struct fat_fid *)fh;
	if (fh_len < 5)
		return NULL;

	switch (fh_type) {
	case FILEID_INO32_GEN_PARENT:
		inode = fat_nfs_get_inode(sb, fid->parent_ino, fid->parent_gen,
						fid->i_pos);
		break;
	}

	return d_obtain_alias(inode);
}

/*
 * Read the directory entries of 'search_clus' and find the entry
 * which contains 'match_ipos' for the starting cluster.If the entry
 * is found, rebuild its inode.
 */
static struct inode *fat_traverse_cluster(struct super_block *sb,
				int search_clus, int match_ipos)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct buffer_head *bh;
	sector_t blknr;
	int parent_ipos, search_ipos;
	int i;
	struct msdos_dir_entry *de;
	struct inode *inode = NULL;
	int iterations = sbi->cluster_size >> sb->s_blocksize_bits;
	blknr = fat_clus_to_blknr(sbi, search_clus);

	do {
		bh = sb_bread(sb, blknr);
		if (!bh) {
			fat_msg(sb, KERN_ERR,
				"NFS:unable to read block(%llu) while traversing cluster(%d)",
				(llu)blknr, search_clus);
			inode = ERR_PTR(-EIO);
			goto out;
		}
		de = (struct msdos_dir_entry *)bh->b_data;
		for (i = 0; i < sbi->dir_per_block; i++) {
			if (de[i].name[0] == FAT_ENT_FREE) {
				/*Reached end of directory*/
				brelse(bh);
				inode = ERR_PTR(-ENODATA);
				goto out;
			}
			if (de[i].name[0] == DELETED_FLAG)
				continue;
			if (de[i].attr == ATTR_EXT)
				continue;
			if (!(de[i].attr & ATTR_DIR))
				continue;
			else {
				search_ipos = fat_get_start(sbi, &de[i]);
				if (search_ipos == match_ipos) {
					/*Success.Now build the inode*/
					parent_ipos = (loff_t)i +
					 (blknr << sbi->dir_per_block_bits);
					inode = fat_build_inode(sb, &de[i],
								parent_ipos);
					brelse(bh);
					goto out;
				}
			}
		}
		brelse(bh);
		blknr += 1;
	} while (--iterations > 0);
out:
	return inode;
}

/*
 * Read the FAT to find the next cluster in the chain
 * corresponding to 'search_clus'.
 */
static int fat_read_next_clus(struct super_block *sb, int search_clus)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	/*bits 31 to 7 give relative sector number*/
	sector_t blknr = search_clus >> 7;
	/*bits 6 to 0 give offset*/
	unsigned int offset = search_clus & 0x7F;
	__le32 *address;
	unsigned int next_cluster;
	struct buffer_head *bh = sb_bread(sb, blknr + sbi->fat_start);
	if (!bh) {
		fat_msg(sb, KERN_ERR,
			"NFS:unable to read block(%llu) for finding the next cluster in FAT chain",
			(llu)blknr);
		return -EIO;
	}
	address = (__le32 *) bh->b_data;
	next_cluster = (le32_to_cpu(address[offset])) & 0x0FFFFFFF;
	brelse(bh);
	return next_cluster;
}

/*
 * Find the parent for a directory that is not currently connected to
 * the filesystem root.
 *
 * On entry, the caller holds child_dir->d_inode->i_mutex.
 */
struct dentry *fat_get_parent(struct dentry *child_dir)
{
	struct super_block *sb = child_dir->d_sb;
	struct buffer_head *dotdot_bh = NULL, *parent_bh = NULL;
	struct msdos_dir_entry *de;
	struct inode *parent_inode = NULL;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	int parent_logstart;
	int search_clus, clus_to_match;
	sector_t blknr;

	if (!fat_get_dotdot_entry(child_dir->d_inode, &dotdot_bh, &de)) {
		parent_logstart = fat_get_start(sbi, de);
		parent_inode = fat_dget(sb, parent_logstart);
		if (parent_inode || sbi->options.nfs != FAT_NFS_NOSTALE_RO)
			goto out;
		if (!parent_logstart)
			/*logstart of dotdot entry is zero if
			* if the directory's parent is root
			*/
			parent_inode = sb->s_root->d_inode;
		else {
			blknr = fat_clus_to_blknr(sbi, parent_logstart);
			parent_bh = sb_bread(sb, blknr);
			if (!parent_bh) {
				fat_msg(sb, KERN_ERR,
					"NFS:unable to read cluster of parent directory");
				goto out;
			}
			de = (struct msdos_dir_entry *) parent_bh->b_data;
			clus_to_match = fat_get_start(sbi, &de[0]);
			search_clus = fat_get_start(sbi, &de[1]);
			if (!search_clus)
				search_clus = sbi->root_cluster;
			brelse(parent_bh);
			do {
				parent_inode =  fat_traverse_cluster(sb,
						search_clus, clus_to_match);
				if (IS_ERR(parent_inode) || parent_inode)
					break;
				search_clus = fat_read_next_clus(sb,
								search_clus);
				if (search_clus < 0)
					break;
			} while (search_clus != FAT_ENT_EOF);
		}
	}
out:
	brelse(dotdot_bh);

	return d_obtain_alias(parent_inode);
}
