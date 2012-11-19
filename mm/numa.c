/*
 * Generic NUMA page table entry support. This code reuses
 * PROT_NONE: an architecture can choose to use its own
 * implementation, by setting CONFIG_ARCH_SUPPORTS_NUMA_BALANCING
 * and not setting CONFIG_ARCH_WANTS_NUMA_GENERIC_PGPROT.
 */
#include <linux/mm.h>

static inline pgprot_t vma_prot_none(struct vm_area_struct *vma)
{
	/*
	 * obtain PROT_NONE by removing READ|WRITE|EXEC privs
	 */
	vm_flags_t vmflags = vma->vm_flags & ~(VM_READ|VM_WRITE|VM_EXEC);

	return pgprot_modify(vma->vm_page_prot, vm_get_page_prot(vmflags));
}

bool pte_numa(struct vm_area_struct *vma, pte_t pte)
{
	/*
	 * For NUMA page faults, we use PROT_NONE ptes in VMAs with
	 * "normal" vma->vm_page_prot protections.  Genuine PROT_NONE
	 * VMAs should never get here, because the fault handling code
	 * will notice that the VMA has no read or write permissions.
	 *
	 * This means we cannot get 'special' PROT_NONE faults from genuine
	 * PROT_NONE maps, nor from PROT_WRITE file maps that do dirty
	 * tracking.
	 *
	 * Neither case is really interesting for our current use though so we
	 * don't care.
	 */
	if (pte_same(pte, pte_modify(pte, vma->vm_page_prot)))
		return false;

	return pte_same(pte, pte_modify(pte, vma_prot_none(vma)));
}

pte_t pte_mknuma(struct vm_area_struct *vma, pte_t pte)
{
	return pte_modify(pte, vma_prot_none(vma));
}

#ifdef CONFIG_ARCH_USES_NUMA_GENERIC_PGPROT_HUGEPAGE
bool pmd_numa(struct vm_area_struct *vma, pmd_t pmd)
{
	/*
	 * See pte_numa() above
	 */
	if (pmd_same(pmd, pmd_modify(pmd, vma->vm_page_prot)))
		return false;

	return pmd_same(pmd, pmd_modify(pmd, vma_prot_none(vma)));
}
#endif

/*
 * The scheduler uses this function to mark a range of virtual
 * memory inaccessible to user-space, for the purposes of probing
 * the composition of the working set.
 *
 * The resulting page faults will be demultiplexed into:
 *
 *    mm/memory.c::do_numa_page()
 *    mm/huge_memory.c::do_huge_pmd_numa_page()
 *
 * This generic version simply uses PROT_NONE.
 */
unsigned long change_prot_numa(struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
	return change_protection(vma, start, end, vma_prot_none(vma), 0);
}
