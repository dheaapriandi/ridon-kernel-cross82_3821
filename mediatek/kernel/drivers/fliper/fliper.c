#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>

#include <linux/timer.h>
#include <linux/jiffies.h>

#define SEQ_printf(m, x...)	    \
 do {			    \
    if (m)		    \
	seq_printf(m, x);	\
    else		    \
	printk(x);	    \
 } while (0)

#define X_ms 200
#define Y_steps (2000/X_ms)
static void enable_fliper(void);
static void disable_fliper(void);
extern unsigned int get_ddr_type(void)__attribute__((weak));

/* define supported DRAM types */
enum
{
  LPDDR2 = 0,
  DDR3_16,
  DDR3_32,
  LPDDR3,
  mDDR,
};

static ssize_t mt_fliper_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    char buf[64];
    int val;
    int ret;
    if (cnt >= sizeof(buf))
        return -EINVAL;

    if (copy_from_user(&buf, ubuf, cnt))
        return -EFAULT;

    buf[cnt] = 0;

    ret = strict_strtoul(buf, 10, (unsigned long*)&val);
    if (ret < 0)
        return ret;
    if(val == 1){
        enable_fliper();
    }else if(val == 0){
        disable_fliper();
    }
    printk(" fliper option: %d\n", val);
    return cnt;

}

static int mt_fliper_show(struct seq_file *m, void *v)
{
    SEQ_printf(m, "----------------------------------------\n");
    return 0;
}
/*** Seq operation of mtprof ****/
static int mt_fliper_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, mt_fliper_show, inode->i_private); 
} 

static const struct file_operations mt_fliper_fops = { 
    .open = mt_fliper_open, 
    .write = mt_fliper_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};
/******* POWER PERF TRANSFORMER *********/
#include <asm/div64.h>
//Cache info
extern unsigned int get_cache_refill(unsigned int cpu);
extern unsigned int get_cache_access(unsigned int cpu);
extern void fliper_pmu_reset(void);
#include <mach/mt_cpufreq.h>

static void mt_power_pef_transfer(void);
static DEFINE_TIMER(mt_pp_transfer_timer, mt_power_pef_transfer, 0, 0);
static int pp_index;

static void mt_power_pef_transfer_work(void);
static DECLARE_DELAYED_WORK(mt_pp_work, mt_power_pef_transfer_work);

//EMI
extern unsigned long long get_mem_bw(void);

static void mt_power_pef_transfer_work()
{
    unsigned long long refill, access, per, t_per, t_refill;
    unsigned long long emi_bw;
    int cpu;
    int cpu_num=0;
    int perf_mode = 0;
    int high_count = 0; int low_count = 0;
    t_refill = 0;
    t_per = 0;

    int depth = 0;
    for_each_online_cpu(cpu){
        cpu_num++;
        refill = (unsigned long long)get_cache_refill(cpu);
        access = (unsigned long long)get_cache_access(cpu);
        if(refill!=0){
            per = refill*100;
            do_div(per, access);
            t_per += per;
        }
        t_refill += refill;

        per > 10 ? high_count++:low_count++;
        //printk(KERN_EMERG"CPU %2d: %3llu%% %10llu/%10llu\n", cpu, per, refill, access);
       // fliper_pmu_reset();
    }
    if(t_per!=0)
        do_div(t_per, cpu_num);
    
    /* 1. Get EMI*/ 
        emi_bw = get_mem_bw();
    /***/

    /*MT POWER-PERF TRANSFER algorithm*/ 
/*    if(t_per > 20 || high_count > low_count){
        perf_mode = 1;
    }*/
    if(emi_bw > 1300)
        perf_mode = 1;

    if(perf_mode == 1){
        if(pp_index == 0)
            mt_soc_dvfs(SOC_DVFS_TYPE_BENCHMARK, 1);
        //pp_index = 0x100;
        pp_index = 1 << Y_steps;
    }else{
        if(pp_index == 1)
            mt_soc_dvfs(SOC_DVFS_TYPE_BENCHMARK, 0);
        pp_index = pp_index >> 1;
    }
    //printk(KERN_EMERG"Rate %10llu%%, %4d\n", t_per, pp_index);

    // refill
    if(t_refill!=0)
        do_div(t_refill, cpu_num);
    //printk(KERN_EMERG"Miss %10llu%%, %4d, %llu\n", t_refill, pp_index, t_per);
    printk(KERN_EMERG"EMI:Rate:count:mode %6llu:%3llu:%6llu:%4d\n", emi_bw, t_per, t_refill, pp_index); 
   
    //printk(KERN_EMERG"======\n");

}
static void enable_fliper()
{
    printk("fliper enable +++\n");
    mod_timer(&mt_pp_transfer_timer, jiffies + msecs_to_jiffies(X_ms));
}
static void disable_fliper()
{
    printk("fliper disable ---\n");
    del_timer(&mt_pp_transfer_timer);
}
static void mt_power_pef_transfer()
{
    mod_timer(&mt_pp_transfer_timer, jiffies + msecs_to_jiffies(X_ms));
    schedule_work(&mt_pp_work);
}
static int __init init_fliper(void)
{
    struct proc_dir_entry *pe;
    int DRAM_Type=get_ddr_type();

    pe = proc_create("fliper", 0664, NULL, &mt_fliper_fops);
    if (!pe)
        return -ENOMEM;

    printk("prepare mt pp transfer: jiffies:%lu-->%lu\n",jiffies, jiffies + msecs_to_jiffies(5000));
    printk("-  next jiffies:%lu >>> %lu\n",jiffies, jiffies + msecs_to_jiffies(X_ms));
    mod_timer(&mt_pp_transfer_timer, jiffies + msecs_to_jiffies(5000));
    return 0;
}
__initcall(init_fliper);
