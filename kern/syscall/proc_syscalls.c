#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <synch.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include "opt-A2.h"
#include <mips/trapframe.h>

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;

#if OPT_A2
  if(p -> parent_alive)
  {
    struct proc *parent = p->parent;
    int index = p -> parent_index;
    parent->children.child_alive[index] = false;
    parent->children.child_ec[index] = exitcode;
    if (parent->children.child_wlk[index] != NULL)
    {
      cv_signal(parent->children.child_wcv[index], parent->children.child_wlk[index]);
    }
  }

  clean_lks(p->children);
  announce_children(p->children);
#else
    /* for now, just include this to keep the compiler from complaining about
	 an unused variable */
    (void)exitcode;
#endif /* OPT_A2 */

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
#if OPT_A2
  *retval = curproc->pid;
  return(0);
#else
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = 1;
  return(0);
#endif /* OPT_A2 */
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

  if (options != 0) {
    return(EINVAL);
  }

#if OPT_A2
  int index = find_child(curproc->children, pid);
  if(index == -1)
  {
    *retval = -1;
    return ESRCH;
  }

  if (curproc->children.child_alive[index])
  {
    curproc->children.child_wlk[index] = lock_create("child waiting lock");
    curproc->children.child_wcv[index] = cv_create("child waiting cv");
    lock_acquire(curproc->children.child_wlk[index]);
    while(curproc->children.child_alive[index])
    {
      cv_wait(curproc->children.child_wcv[index], curproc->children.child_wlk[index]);
    }
    lock_release(curproc->children.child_wlk[index]);
  }

  // if child is died before, exit code is already set in the array
  exitstatus = _MKWAIT_EXIT(curproc->children.child_ec[index]);

#else
    /* for now, just pretend the exitstatus is 0 */
    exitstatus = 0;
#endif /* OPT_A2 */

  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2
void
enter_child_process(void *tf, unsigned long as)
{
  //copy trapfram
  struct trapframe *new_tf = kmalloc(sizeof(struct trapframe));
  memcpy(new_tf, tf, sizeof(struct trapframe));

  //set return values
  new_tf->tf_v0 = 0;  //return value for fork in child
  new_tf->tf_a3 = 0;

  //pc+=4
  new_tf->tf_epc += 4;

  (void)as;
  mips_usermode(new_tf);
}

int
sys_fork(struct trapframe *tf, pid_t *retval)
{
  //create child process
  struct proc *child = proc_create_runprogram("child_proc");
  KASSERT(child != NULL);

  //copy address space
  int err = as_copy(curproc->p_addrspace, &child->p_addrspace);
  if(err != 0) 
  {
    proc_destroy(child);
    return ENOMEM;
  }

  //create new thread
  thread_fork("child_thread", child, enter_child_process, tf, 1);

  //set child-parent relationship
  child->parent = curproc;
  child->parent_alive = true;
  int new_ind = get_empty_index(curproc->children);
  child->parent_index = new_ind;
  curproc->children.child_pids[new_ind] = child->pid;
  curproc->children.child_alive[new_ind] = true;
  curproc->children.child_procs[new_ind] = child;
  
  
  *retval = child->pid;
  return 0;
}
#endif
