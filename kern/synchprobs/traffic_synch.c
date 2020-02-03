#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
struct intersection
{
  //trail 1
  volatile bool t1_taken;
  volatile Direction t1_ori;
  volatile Direction t1_des;

  //trail 2
  volatile bool t2_taken;
  volatile Direction t2_ori;
  volatile Direction t2_des;
};

static struct intersection* t_intersection;
static struct lock* intersection_lk;
static struct cv* n_veh_allowed;
static struct cv* s_veh_allowed;
static struct cv* e_veh_allowed;
static struct cv* w_veh_allowed;

static int dir_por[4] = {0,0,0,0};
static int dir_wait[4] = {0,0,0,0};

static bool
right_turn(Direction ori, Direction des)
{
  if ((int)ori == 0 && (int)des == 3)
  {
    return true;
  }
  else if ((int)ori == (int)des + 1)
  {
    return true;
  }

  return false;
}

static bool
trail_allow(struct intersection* cur_int, Direction ori, Direction des)
{
  if(cur_int->t1_taken && cur_int->t2_taken) 
  {
    return false;
  }

  if(!cur_int->t1_taken && !cur_int->t2_taken)
  {
    return true;
  }

  Direction tem_ori, tem_des;
  if(cur_int->t1_taken)
  {
    tem_ori = cur_int->t1_ori;
    tem_des = cur_int->t1_des;
  }
  else
  {  
    tem_ori = cur_int->t2_ori;
    tem_des = cur_int->t2_des;
  }
  
  if((tem_ori == ori) // entered the intersection from the same direction
    ||(tem_ori == des && tem_des == ori)  // going in opposite directions
    ||((right_turn(tem_ori, tem_des) || right_turn(ori, des)) && tem_des != des)) // one vehicle is right turn and they have different des
  {
    return true;
  }
  
  return false;
}

// static int
// next_dir(int dp[], int dw[])
// {
//   int max = 0;
//   int pos = 0;
//   for(int i = 0; i < 4; i++)
//   {
//     if(dp[i] > max && dw[i] > 0)
//     {
//       max = dp[i];
//       pos = i;
//     }
//   }

//   for(int i = 0; i < 4; i++)
//   {
//     if(i == pos)
//     {
//       dp[i] = 0;
//       dw[i]--;
//     }
//     else
//     {
//       dp[i]++;
//     }
//   }
//   return pos;
// }


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  t_intersection = kmalloc(sizeof(struct intersection));
  t_intersection->t1_taken = false;
  t_intersection->t2_taken = false;

  intersection_lk = lock_create("intersectionLock");
  n_veh_allowed = cv_create("NorthVehicleCV");
  s_veh_allowed = cv_create("SouthVehicleCV");
  e_veh_allowed = cv_create("EastVehicleCV");
  w_veh_allowed = cv_create("WestVehicleCV");

  if (intersection_lk == NULL) {
    panic("could not create locks");
  }

  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersection_lk != NULL);
  lock_destroy(intersection_lk);
  cv_destroy(n_veh_allowed);
  cv_destroy(s_veh_allowed);
  cv_destroy(e_veh_allowed);
  cv_destroy(w_veh_allowed);
  kfree(t_intersection);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersection_lk != NULL);
  lock_acquire(intersection_lk);
  dir_wait[(int)origin]++;
  while(!trail_allow(t_intersection, origin, destination))
  {
    int ori_num = (int)origin;
    KASSERT(ori_num == 0 || ori_num == 1|| ori_num == 2 || ori_num == 3);
    if(ori_num == 0)
    {
      cv_wait(n_veh_allowed, intersection_lk);   
    }
    else if (ori_num == 1)
    {
      cv_wait(e_veh_allowed, intersection_lk);   
    }
    else if (ori_num == 2)
    {
      cv_wait(s_veh_allowed, intersection_lk);   
    }
    else
    {
      cv_wait(w_veh_allowed, intersection_lk);
    }
  }
  
  dir_wait[(int)origin]--;
  dir_por[(int)origin] = 0;
  if(!t_intersection->t1_taken)
  {
    t_intersection->t1_taken = true;
    t_intersection->t1_ori = origin;
    t_intersection->t1_des = destination;
  }
  else
  {
    t_intersection->t2_taken = true;
    t_intersection->t2_ori = origin;
    t_intersection->t2_des = destination;
  }
  lock_release(intersection_lk);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  // (void)origin;  /* avoid compiler complaint about unused parameter */
  // (void)destination; /* avoid compiler complaint about unused parameter */
  // KASSERT(intersectionSem != NULL);
  // V(intersectionSem);

  KASSERT(intersection_lk != NULL);
  lock_acquire(intersection_lk);
  if(t_intersection->t1_taken && t_intersection->t1_ori == origin && t_intersection->t1_des == destination)
  {
    t_intersection->t1_taken = false;
  }
  else
  {
     t_intersection->t2_taken = false;
  }

  int max = -1;
  int pos = -1;
  for(int i = 0; i < 4; i++)
  {
    if(dir_por[i] > max && dir_wait[i] > 0)
    {
      //KASSERT(false);
      max = dir_por[i];
      pos = i;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    if (i != pos && dir_wait[i] > 0)
    {
      dir_por[i]++;
    }
  }

  lock_release(intersection_lk);
  if (pos == 0)
  {
    cv_signal(n_veh_allowed, intersection_lk);
  }
  else if (pos == 1)
  {
    cv_signal(e_veh_allowed, intersection_lk);
  }
  else if (pos == 2)
  {
    cv_signal(s_veh_allowed, intersection_lk);
  }
  else if (pos == 3)
  {
    cv_signal(w_veh_allowed, intersection_lk);
  }
}
