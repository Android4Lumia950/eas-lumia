/*
 * Copyright (C) 2014 Davidlohr Bueso.
 */
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/vmacache.h>

static bool vmacache_valid_mm(struct mm_struct *mm)
{
	return current->mm == mm && !(current->flags & PF_KTHREAD);
}

void vmacache_update(unsigned long addr, struct vm_area_struct *newvma)
{
	if (vmacache_valid_mm(newvma->vm_mm))
		current->vmacache[VMACACHE_HASH(addr)] = newvma;
}

static bool vmacache_valid(struct mm_struct *mm)
{
	struct task_struct *curr;

	if (!vmacache_valid_mm(mm))
		return false;

	curr = current;
	if (mm->vmacache_seqnum != curr->vmacache_seqnum) {
		/*
		 * First attempt will always be invalid, initialize
		 * the new cache for this task here.
		 */
		curr->vmacache_seqnum = mm->vmacache_seqnum;
		vmacache_flush(curr);
		return false;
	}
	return true;
}

struct vm_area_struct *vmacache_find(struct mm_struct *mm, unsigned long addr)
{
	int i;

	if (!vmacache_valid(mm))
		return NULL;

	for (i = 0; i < VMACACHE_SIZE; i++) {
		struct vm_area_struct *vma = current->vmacache[i];

		if (vma && vma->vm_start <= addr && vma->vm_end > addr) {
			BUG_ON(vma->vm_mm != mm);
			return vma;
		}
	}

	return NULL;
}

#ifndef CONFIG_MMU
struct vm_area_struct *vmacache_find_exact(struct mm_struct *mm,
					   unsigned long start,
					   unsigned long end)
{
	int i;

	if (!vmacache_valid(mm))
		return NULL;

	for (i = 0; i < VMACACHE_SIZE; i++) {
		struct vm_area_struct *vma = current->vmacache[i];

		if (vma && vma->vm_start == start && vma->vm_end == end)
			return vma;
	}

	return NULL;
}
#endif
