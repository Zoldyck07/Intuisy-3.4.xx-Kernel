/*
* V(R) I/O Scheduler
*
* Copyright (C) 2007 Aaron Carroll <aaronc@gelato.unsw.edu.au>
*
*
* The algorithm:
*
* The next request is decided based on its distance from the last
* request, with a multiplicative penalty of `rev_penalty' applied
* for reversing the head direction. A rev_penalty of 1 means SSTF
* behaviour. As this variable is increased, the algorithm approaches
* pure SCAN. Setting rev_penalty to 0 forces SCAN.
*
* Async and synch requests are not treated seperately. Instead we
* rely on deadlines to ensure fairness.
*
*/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/version.h>

#include <asm/div64.h>

enum { ASYNC, SYNC };

/* Tunables */
static const int sync_read_expire = HZ / 2;	/* max time before a sync read is submitted. */
static const int sync_write_expire = 2 * HZ;	/* max time before a sync write is submitted. */

static const int async_read_expire = 4 * HZ;	/* ditto for async, these limits are SOFT! */
static const int async_write_expire = 16 * HZ;	/* ditto for async, these limits are SOFT! */

static const int writes_starved = 2;	/* max times reads can starve a write */
static const int fifo_batch = 8;	/* # of sequential requests treated as one
by the above parameters. For throughput. */

/* Elevator data */
struct sio_data {
/* Request queues */
struct list_head fifo_list[2][2];

/* Attributes */
unsigned int batched;
unsigned int starved;

/* Settings */
int fifo_expire[2][2];
int fifo_batch;
int writes_starved;
};

static void
sio_merged_requests(struct request_queue *q, struct request *rq,
struct request *next)
{
/*
* If next expires before rq, assign its expire time to rq
* and move into next position (next will be deleted) in fifo.
*/
if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist)) {
if (time_before(rq_fifo_time(next), rq_fifo_time(rq))) {
list_move(&rq->queuelist, &next->queuelist);
rq_set_fifo_time(rq, rq_fifo_time(next));
}
}

/* Delete next request */
rq_fifo_clear(next);
}

static void
vr_add_request(struct request_queue *q, struct request *rq)
{
struct sio_data *sd = q->elevator->elevator_data;
const int sync = rq_is_sync(rq);
const int data_dir = rq_data_dir(rq);
/*
* We might be deleting our cached next request.
* If so, find its sucessor.
*/

/*
* add rq to rbtree and fifo
*/
static void
vr_add_request(struct request_queue *q, struct request *rq)
struct request *next)
{
struct vr_data *vd = vr_get_data(q);
const int dir = rq_is_sync(rq);
}
/*
* If next expires before rq, assign its expire time to rq
* and move into next position (next will be deleted) in fifo.
*/
if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist)) {
if (time_before(rq_fifo_time(next), rq_fifo_time(rq))) {
list_move(&rq->queuelist, &next->queuelist);
rq_set_fifo_time(rq, rq_fifo_time(next));
}
}

/* Delete next request */
rq_fifo_clear(next);
}

/*
* Add request to the proper fifo list and set its
* expire time.
*/
rq_set_fifo_time(rq, jiffies + sd->fifo_expire[sync][data_dir]);
list_add_tail(&rq->queuelist, &sd->fifo_list[sync][data_dir]);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
static int
vr_queue_empty(struct request_queue *q)
{
struct vr_data *vd = vr_get_data(q);
return RB_EMPTY_ROOT(&vd->sort_list);
}
#endif

static void
vr_exit_queue(struct elevator_queue *e)
{
struct vr_data *vd = e->elevator_data;
BUG_ON(!RB_EMPTY_ROOT(&vd->sort_list));
kfree(vd);
}

/*
* initialize elevator private data (vr_data).
*/
static void *vr_init_queue(struct request_queue *q)
{
struct vr_data *vd;

vd = kmalloc_node(sizeof(*vd), GFP_KERNEL | __GFP_ZERO, q->node);
if (!vd)
return NULL;

INIT_LIST_HEAD(&vd->fifo_list[SYNC]);
INIT_LIST_HEAD(&vd->fifo_list[ASYNC]);
vd->sort_list = RB_ROOT;
vd->fifo_expire[SYNC] = sync_expire;
vd->fifo_expire[ASYNC] = async_expire;
vd->fifo_batch = fifo_batch;
vd->rev_penalty = rev_penalty;
return vd;
}

/*
* sysfs parts below
*/

static ssize_t
vr_var_show(int var, char *page)
{
return sprintf(page, "%d\n", var);
}

static ssize_t
vr_var_store(int *var, const char *page, size_t count)
{
*var = simple_strtol(page, NULL, 10);
return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV) \
static ssize_t __FUNC(struct elevator_queue *e, char *page) \
{ \
struct vr_data *vd = e->elevator_data; \
int __data = __VAR; \
if (__CONV) \
__data = jiffies_to_msecs(__data); \
return vr_var_show(__data, (page)); \
}
SHOW_FUNCTION(vr_sync_expire_show, vd->fifo_expire[SYNC], 1);
SHOW_FUNCTION(vr_async_expire_show, vd->fifo_expire[ASYNC], 1);
SHOW_FUNCTION(vr_fifo_batch_show, vd->fifo_batch, 0);
SHOW_FUNCTION(vr_rev_penalty_show, vd->rev_penalty, 0);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV) \
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count) \
{ \
struct vr_data *vd = e->elevator_data; \
int __data; \
int ret = vr_var_store(&__data, (page), count); \
if (__data < (MIN)) \
__data = (MIN); \
else if (__data > (MAX)) \
__data = (MAX); \
if (__CONV) \
*(__PTR) = msecs_to_jiffies(__data); \
else \
*(__PTR) = __data; \
return ret; \
}
STORE_FUNCTION(vr_sync_expire_store, &vd->fifo_expire[SYNC], 0, INT_MAX, 1);
STORE_FUNCTION(vr_async_expire_store, &vd->fifo_expire[ASYNC], 0, INT_MAX, 1);
STORE_FUNCTION(vr_fifo_batch_store, &vd->fifo_batch, 0, INT_MAX, 0);
STORE_FUNCTION(vr_rev_penalty_store, &vd->rev_penalty, 0, INT_MAX, 0);
#undef STORE_FUNCTION

#define DD_ATTR(name) \
__ATTR(name, S_IRUGO|S_IWUSR, vr_##name##_show, \
vr_##name##_store)

static struct elv_fs_entry vr_attrs[] = {
DD_ATTR(sync_expire),
DD_ATTR(async_expire),
DD_ATTR(fifo_batch),
DD_ATTR(rev_penalty),
__ATTR_NULL
};

static struct elevator_type iosched_vr = {
.ops = {
.elevator_merge_fn = vr_merge,
.elevator_merged_fn = vr_merged_request,
.elevator_merge_req_fn = vr_merged_requests,
.elevator_dispatch_fn = vr_dispatch_requests,
.elevator_add_req_fn = vr_add_request,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
.elevator_queue_empty_fn = vr_queue_empty,
#endif
.elevator_former_req_fn = elv_rb_former_request,
.elevator_latter_req_fn = elv_rb_latter_request,
.elevator_init_fn = vr_init_queue,
.elevator_exit_fn = vr_exit_queue,
},

.elevator_attrs = vr_attrs,
.elevator_name = "vr",
.elevator_owner = THIS_MODULE,
};

static int __init vr_init(void)
{
elv_register(&iosched_vr);

return 0;
}

static void __exit vr_exit(void)
{
elv_unregister(&iosched_vr);
}

module_init(vr_init);
module_exit(vr_exit);

MODULE_AUTHOR("Aaron Carroll");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V(R) IO scheduler");
