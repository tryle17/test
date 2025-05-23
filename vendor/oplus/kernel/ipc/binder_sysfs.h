#ifndef _OPLUS_BINDER_SYSFS_H_
#define _OPLUS_BINDER_SYSFS_H_

extern unsigned int g_async_ux_enable;
extern unsigned int g_set_last_async_ux;
extern unsigned int g_set_async_ux_after_pending;
extern unsigned int unset_ux_match_t;

extern int sync_insert_queue;
extern int fg_list_dynamic_enable;
extern int fg_list_async_first;
extern int get_random_binder_task;

int oplus_binder_sysfs_init(void);
void oplus_binder_sysfs_deinit(void);

extern struct task_struct *find_task_by_vpid(pid_t vnr);
extern void set_task_async_ux_enable(pid_t pid, int enable);
extern bool get_task_async_ux_enable(pid_t pid);
extern void get_all_tasks_async_ux_enable(void);

#endif
