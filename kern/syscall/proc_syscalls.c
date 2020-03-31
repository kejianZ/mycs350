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
#include "opt-A3.h"
#include <vfs.h>
#include <kern/fcntl.h>
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
 
  announce_children(p->children);
  clean_children_info(p->children);
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

#if OPT_A3
void tsys_kill(int exitcode)
{
  (void) exitcode;
  struct addrspace *as;
  struct proc *p = curproc;


  if(p -> parent_alive)
  {
    struct proc *parent = p->parent;
    int index = p -> parent_index;
    parent->children.child_alive[index] = false;
    parent->children.child_fatal[index] = true;
    if (parent->children.child_wlk[index] != NULL)
    {
      cv_signal(parent->children.child_wcv[index], parent->children.child_wlk[index]);
    }
  }
 
  announce_children(p->children);
  clean_children_info(p->children);

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
#endif 


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
#if OPT_A3
  if(curproc->children.child_fatal[index])
  {
    exitstatus = _MKWAIT_SIG(0);
  }
  else
  {
    exitstatus = _MKWAIT_EXIT(curproc->children.child_ec[index]);
  }
#else
  // if child is died before, exit code is already set in the array
  exitstatus = _MKWAIT_EXIT(curproc->children.child_ec[index]);
#endif

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
  struct trapframe *new_tf;
  new_tf = (struct trapframe*) tf;
  struct trapframe cp_tf = *new_tf;
  kfree(tf);

  //set return values
  cp_tf.tf_v0 = 0;  //return value for fork in child
  cp_tf.tf_a3 = 0;

  //pc+=4
  cp_tf.tf_epc += 4;

  (void)as;
  mips_usermode(&cp_tf);
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
  struct trapframe *new_tf = kmalloc(sizeof(struct trapframe));
  memcpy(new_tf, tf, sizeof(struct trapframe));
  thread_fork("child_thread", child, enter_child_process, new_tf, 1);

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

#if OPT_A2
// A2b
int sys_execv(const char *program, char **args)
{
  // count argv # and copy them into kernel
  int argc = 0;
  for(int i = 0; args[i] != NULL; i++)
  {
    argc++;
  }

  // malloc for argv array
  char** hp_argv = kmalloc((argc + 1) * sizeof(char *));
  KASSERT(hp_argv != NULL);

  int* arg_lens = kmalloc((argc) * sizeof(int));
  // copy args into argv
  for(int i = 0; i < argc; i++)
  {
    size_t arglen = (strlen(args[i]) + 1) * sizeof(char);
    arg_lens[i] = arglen;
    hp_argv[i] = kmalloc(arglen);
    KASSERT(hp_argv[i] != NULL);
    int copy_r = copyin((const_userptr_t)args[i], (void *)hp_argv[i], arglen);
    KASSERT(copy_r == 0);
  }
  hp_argv[argc] = NULL;

  // copy prog path into kernel
  size_t pathlen = (strlen(program) + 1) * sizeof(char);
  char* hp_path = kmalloc(pathlen);
  KASSERT(hp_path != NULL);
  copyin((const_userptr_t)program, (void *)hp_path, pathlen);

  
  ///////////////////////////////////////////////////////////////////////////////////
  // copy from runprogram
  struct addrspace *as;
	struct vnode *v;
  vaddr_t entrypoint, stackptr;
  int result;
  /* Open the file. */
	result = vfs_open(hp_path, O_RDONLY, 0, &v);
	if (result) {
		return result;
	}

  /* Create a new address space. */
	as = as_create();
	if (as ==NULL) {
		vfs_close(v);
		return ENOMEM;
	}

  /* Switch to it and activate it. */
  struct addrspace* old_as = curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);

  /* Define the user stack in the address space */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  vaddr_t temp_ptr = stackptr;
  vaddr_t* arg_addr = kmalloc((argc + 1) * sizeof(vaddr_t));  //address of arguements in stack

  arg_addr[argc] = (vaddr_t) NULL;
  for(int i = argc - 1; i >= 0 ; i--)
  {
    size_t arglen = arg_lens[i];
    temp_ptr -= arglen;
    arg_addr[i] = temp_ptr;
    int copy_r = copyout((void *)hp_argv[i], (userptr_t)temp_ptr, arglen);
    KASSERT(copy_r == 0);
  }

  temp_ptr = stackptr - ROUNDUP(stackptr-temp_ptr, 4);
  for(int i = argc; i >= 0; i--)
  {
    temp_ptr -= 4;
    int copy_r = copyout((void *)&arg_addr[i], (userptr_t)temp_ptr, 4);
    KASSERT(copy_r == 0);
  }

  // delete old address space
  as_destroy(old_as);

  // delete arg related arrays on heap
  kfree(arg_addr);
  for(int i = 0; i < argc; i++)
  {
    kfree(hp_argv[i]);
  }
  kfree(hp_argv);
  kfree(arg_lens);
  kfree(hp_path);

  // enter the new process
  enter_new_process(argc, (userptr_t)temp_ptr,
			  stackptr - ROUNDUP(stackptr - temp_ptr, 8), entrypoint);
	
	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;
}
#endif
