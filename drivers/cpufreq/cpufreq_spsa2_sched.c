/*
 *  drivers/cpufreq/cpufreq_ondemand.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/sched/cpufreq.h>

// #include "cpufreq_ondemand.h"
#include "cpufreq_governor.h"
#include "cpufreq_spsa2.h"
#include <linux/random.h>
#include <linux/string.h>


#include <linux/sched/signal.h>
#include <linux/sched.h>

/* On-demand governor macros */
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(1)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)

static struct od_ops od_ops;

static unsigned int default_powersave_bias;

/*
 * Not all CPUs want IO time to be accounted as busy; this depends on how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (android.com) claims this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) and later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
			boot_cpu_data.x86 == 6 &&
			boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_delay_us,
 * freq_lo, and freq_lo_delay_us in percpu area for averaging freqs.
 */
static unsigned int generic_powersave_bias_target(struct cpufreq_policy *policy,
		unsigned int freq_next, unsigned int relation)
{
	unsigned int freq_req, freq_reduc, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index;
	unsigned int delay_hi_us;
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct od_policy_dbs_info *dbs_info = to_dbs_info(policy_dbs);
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	struct cpufreq_frequency_table *freq_table = policy->freq_table;

	if (!freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_delay_us = 0;
		return freq_next;
	}

	index = cpufreq_frequency_table_target(policy, freq_next, relation);
	freq_req = freq_table[index].frequency;
	freq_reduc = freq_req * od_tuners->powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = cpufreq_table_find_index_h(policy, freq_avg);
	freq_lo = freq_table[index].frequency;
	index = cpufreq_table_find_index_l(policy, freq_avg);
	freq_hi = freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_delay_us = 0;
		return freq_lo;
	}
	delay_hi_us = (freq_avg - freq_lo) * dbs_data->sampling_rate;
	delay_hi_us += (freq_hi - freq_lo) / 2;
	delay_hi_us /= freq_hi - freq_lo;
	dbs_info->freq_hi_delay_us = delay_hi_us;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_delay_us = dbs_data->sampling_rate - delay_hi_us;
	return freq_hi;
}

static void spsa2_sched_powersave_bias_init(struct cpufreq_policy *policy)
{
	struct od_policy_dbs_info *dbs_info = to_dbs_info(policy->governor_data);

	dbs_info->freq_lo = 0;
}

/// SPSA2 functionality
//////////////////////////////////////////////////////////////////
#define FREQ_MAX_AMOUNT 16

static struct spsa2_policy_dbs_info clusters_data[2];


static unsigned int freq_1[FREQ_MAX_AMOUNT] = {
	500000,
	774000,
	875000,
	975000,
	1075000,
	1175000,
	1275000,
	1375000,
	1500000,
	1618000,
	1666000,
	1733000,
	1800000,
	1866000,
	1933000,
	2000000
};

static unsigned int freq_2[FREQ_MAX_AMOUNT] = {
	774000,
	835000,
	919000,
	1002000,
	1085000,
	1169000,
	1308000,
	1419000,
	1530000,
	1670000,
	1733000,
	1796000,
	1860000,
	1923000,
	1986000,
	2050000
};

static unsigned int en_1[FREQ_MAX_AMOUNT] = {
	391000, 
	303618, 
	285714, 
	285744, 
	290605, 
	302128, 
	311294, 
	326036, 
	348867, 
	364339, 
	372449, 
	384362, 
	404278, 
	430171, 
	443870, 
	450200
};

static unsigned int en_2[FREQ_MAX_AMOUNT] = {
	734496, 
	735090, 
	768770, 
	793713, 
	839724, 
	899829, 
	996407, 
	1074419, 
	1159412, 
	1255868, 
	1347721, 
	1378229, 
	1449516, 
	1515965, 
	1550755, 
	1582098
};


static unsigned int get_freq_amount() 
{ 
	return FREQ_MAX_AMOUNT;
}

static unsigned int* get_freq_array(unsigned int cluster)
{
    unsigned int* freq;
    freq = cluster < 1 ? freq_1 : freq_2;
    
    if (cluster > 1)
        pr_warn("gov spsa2 <get_freq_array> WRONG CLUSTER: %u \n", cluster);
    
    return freq;	
}

static unsigned int* get_en_array(unsigned int cluster)
{
    unsigned int* en;
    en = cluster < 1 ? en_1 : en_2;
    
    if (cluster > 1)
        pr_warn("gov spsa2 <get_en_array> WRONG CLUSTER: %u \n", cluster);
    
    return en;	
}

static signed int generate_delta(void)
{
    int rand;
    get_random_bytes(&rand, sizeof(rand));
	
    //pr_warn("gov spsa2, minus, random delta %d \n", rand);
    if (rand & 0x1)
    {
        //pr_warn("gov spsa2, minus, random delta %d \n", rand);
        return -1;
    }
    
    //pr_warn("gov spsa2, plus, random delta %d \n", rand);

    return 1;
}

static int find(unsigned int* freq, int freq_amount, unsigned int val)
{
    int i;
    for (i = 0; i < freq_amount; i++) {
        if (val == freq[i]) {
            return i;
        }
    }

    return -1;
}

static unsigned int find_closest(unsigned int frequency, unsigned int* freq, int freq_amount)
{
    unsigned int closest_ind;
    int i;

    closest_ind = 0;

    for (i = 0; i < freq_amount; i++) {
        if (abs((int)freq[i] - (int)frequency) < abs((int)frequency - (int)freq[closest_ind])) {
            closest_ind = i;
        }
    }

    return closest_ind;
}

static int calculate_functional(unsigned int current_load, int index, unsigned int* freq, unsigned int* en, int freq_amount, unsigned int target_load)
{
    unsigned int target_index = 0;
	unsigned int target_freq = 0;
	unsigned int current_freq = 0;
    unsigned int volume = 0;
    
    if (index < 0)
    {
            index = 0;
    }
    if (index > freq_amount - 1)
    {
            index = freq_amount - 1;
    }

    current_freq = freq[index];
	
	volume = current_freq * current_load;
	
	if(current_load <= target_load)
	{
		// no higher than target - trying to optimize consumption
		int min_index = freq_amount - 1;
		int i = 0;
		
		for(i = 0; i < freq_amount; i++) // searching amongst all frequencies for optimum
		{
			unsigned int projected_load = volume / freq[i];
			if(projected_load <= target_load && en[i] < en[min_index])
			{
				min_index = i;
			}
	    }
		
		target_freq = freq[min_index];
        target_index = min_index;
	}
	else
	{
		// higher than target => trying to meet the requirements
		target_index = find_closest(volume / target_load, freq, freq_amount);
        
        if ((current_load >= 100) && (target_index < freq_amount - 2))
        {
            target_index += 1;
        }

        target_freq = freq[target_index];
	}

	return abs(index - (int)target_index);
}

static int determine_new_freq(struct cpufreq_policy* policy, struct spsa2_policy_dbs_info* dbs_info, struct od_dbs_tuners *od_tuners, unsigned int cluster, unsigned int load)
{
	unsigned int* freq;
	//freq = policy->cpu < 6 ? freq_1 : freq_2;
	unsigned int* en;
	//en = policy->cpu < 6 ? en_1 : en_2;
	
	unsigned int current_freq;
	int index;

    int model;

	int new_index;

    unsigned int next_freq, target_load, log_print;
    
    int freq_amount;
    int beta;
    int alpha;
    int difference, gradient;
    // Have to add int alpha and int beta to <drivers/cpufreq/cpufreq_governor.h>, struct od_dbs_tuners
    alpha = od_tuners->alpha;
    beta = od_tuners->beta;
    target_load = od_tuners->target_load;
    log_print = od_tuners->log_print;
    
    //freq = policy->cpu < 4 ? freq_1 : freq_2;	
    //en = policy->cpu < 4 ? en_1 : en_2;
    //freq_amount = policy->cpu < 4 ? FREQ_MAX_AMOUNT_1 : FREQ_MAX_AMOUNT_2;
    
    freq = get_freq_array(cluster);
    en = get_en_array(cluster);
    freq_amount = get_freq_amount();
        
    current_freq = policy->cur;
    
    index = find(freq, freq_amount, current_freq);

    if(index < 0 || index > freq_amount - 1)
	{   
        if (log_print)
		    pr_warn("gov spsa2, frequency %u is not found for cpu %u \n", current_freq, policy->cpu);
		index = 0;
	}
	
//	if(dbs_info->requested_freq != 0 && dbs_info->requested_freq != current_freq)
//	{
//        int index;
//		// we wait CPU to switch
//		//dbs_info->load_sum += load;
//		//dbs_info->load_count += 1;
//
//        //TODO may have trash in dbs_info->requested_freq
//        for(index = 0; index < freq_amount; index++)
//        {
//            if (dbs_info->requested_freq == freq[index])
//            {
//                pr_warn("gov spsa2, waiting cpu %u to switch from %u to %u \n", policy->cpu, current_freq, dbs_info->requested_freq);
//		        return dbs_info->requested_freq;
//            }
//        }
//
//		pr_warn("gov spsa2, waiting cpu %u to switch from %u to %u, but it is wrong freq (set %u freq as target) \n", policy->cpu, current_freq, dbs_info->requested_freq, freq[freq_amount / 2]);
//        dbs_info->requested_freq = freq[freq_amount / 2]; 		
//        return dbs_info->requested_freq;
//	}
	
    model = 0; //calculate_functional(load, index, freq, en, freq_amount, target_load);
    new_index = index;
    //////////////////////////////03.02.2023
    /// test 1 - only phase 0, calculate to functionals
    /// test 2 - 3 phase:
    ///// 0 - calculate minus  (and run freq)
    ///// 1 - calculate plus   (and run freq)
    ///// 2 - calculate gradient from prev 2 calcs and run the freq obtained
 
    if (dbs_info->old_index < 0 || dbs_info->old_index > (freq_amount - 1))
        dbs_info->old_index = index;
    

    switch(dbs_info->phase) 
    {

        case 0:
            
            // minus measurement
            dbs_info->delta = generate_delta();

            dbs_info->old_index = index;

            dbs_info->minus_model = calculate_functional(load, index, freq, en, freq_amount, target_load);               
            
            // plus measurement
            new_index = dbs_info->old_index + dbs_info->delta * beta;
            
            if (log_print)            
                pr_warn("gov_spsa2, result cpu %u p:%u -- load: %u, t_load: %u, del: %d, old_i: %d, new_i: %d\n", policy->cpu, dbs_info->phase, load, target_load, dbs_info->delta, index, new_index);            
            
            dbs_info->phase += 1;
            break;
        
        case 1:
            dbs_info->plus_model = calculate_functional(load, index, freq, en, freq_amount, target_load);            
            
            difference = dbs_info->plus_model - dbs_info->minus_model;
            if (abs(difference) == 1)
            {
                difference *= 2;
            }

            if ((load > target_load) && (difference == 0))
            {
                difference = -2;
                dbs_info->delta = 1;
            }

            gradient = (dbs_info->delta * alpha * (difference)) / (beta);

            if ((load > target_load) && (gradient >= 0))
            {
                int old_grad;
                old_grad = gradient;

                gradient = -1;
                
                if (dbs_info->old_index <= 0)
                    gradient = -2;
                
                if (log_print)
                    pr_warn("spsa2 gradient error, cpu %u: load: %u, t_load: %u, grad: %d, new_grad: %d \n", policy->cpu, load, target_load, old_grad, gradient);
            }

            if ((load < target_load) && (gradient < 0))
            {
                int old_grad;
                old_grad = gradient;

                gradient = 1;

                if (log_print)
                    pr_warn("spsa2 gradient error, cpu %u: load: %u, t_load: %u, grad: %d, new_grad: %d \n", policy->cpu, load, target_load, old_grad, gradient);
            }
            
            new_index = dbs_info->old_index - gradient; 
            
            if (log_print)
                pr_warn("gov_spsa2, result cpu %u p:%u -- load: %u, t_load: %u,  plus: %d, minus: %d, al: %d, bt: %d, del: %d, grad: %d, old_i: %d, new_i: %d\n", policy->cpu, dbs_info->phase, load, target_load, dbs_info->plus_model, dbs_info->minus_model, alpha, beta, dbs_info->delta, gradient, dbs_info->old_index, new_index);


            dbs_info->phase = 0;
            break;


        default :
            dbs_info->phase = 0;
    }    
    
    if (new_index >= freq_amount)
        new_index = freq_amount - 1;
    
    if (new_index < 0)
        new_index = 0;

    next_freq = freq[new_index];
    dbs_info->requested_freq = next_freq;
    
    return next_freq;
}

// SPSA2 functionality ended

// ================================ SPSA2_SCHED ================================ //
#define MAX_PROC_NUM 14

typedef enum app_type 
{
    AUDIO,
    VIDEO,
    CAMERA,
    TYPE,
    GAME,
    ANOTHER
} app_type;

struct task_cons {
	int pid;
	char comm[TASK_COMM_LEN];
	unsigned int cpu;
	unsigned long pcount;                 // times process have run on this CPU 
	unsigned long long exec_start;        // starting time of the process in the last scheduling tick period
	unsigned long long sum_exec_runtime;  // total runtime of the process till now
	unsigned long long vruntime;		  // virtual runtime
};

struct spsa2_sched_staff {
	unsigned int phase;
	unsigned long long dvfs_iter_count;

	unsigned int left_snapshot_len;
	unsigned int right_snapshot_len;

	struct task_cons left_snapshot[MAX_PROC_NUM];
	struct task_cons right_snaphot[MAX_PROC_NUM];
};

// prosess' cmd
static char aimp_comm[TASK_COMM_LEN] = "com.aimp.player";
static char mx_player_comm[TASK_COMM_LEN] = ".videoplayer.ad";
static char note_comm[TASK_COMM_LEN] = "appmindlab.nano";
static char hill_climb_comm[TASK_COMM_LEN] = "rsoft.hillclimb";
static char open_camera_comm[TASK_COMM_LEN] = "orge.opencamera";
static char audio_service_mediatek[TASK_COMM_LEN] = "ervice.mediatek";
static char audioserver[TASK_COMM_LEN] = "audioserver";
static char mediaserver[TASK_COMM_LEN] = "mediaserver";
static char mediaswcodec[TASK_COMM_LEN] = "in/mediaswcodec";
static char graphics_service[TASK_COMM_LEN] = "ser@2.1-service";
static char mediatek_service[TASK_COMM_LEN] = ".pq@2.2-service";
static char inputmethod[TASK_COMM_LEN] = "putmethod.latin";
static char camerahalserver[TASK_COMM_LEN] = "camerahalserver";
static char cameraserver[TASK_COMM_LEN] = "cameraserver";


static struct spsa2_sched_staff spsa2_sched_staff;

static int is_needed_task(struct task_struct *task) 
{
	char *comm = task->comm;

	if      (strncmp(comm, aimp_comm, TASK_COMM_LEN) == 0)
		return 1;
	else if (strncmp(comm, mx_player_comm, TASK_COMM_LEN) == 0)
		return 1;
	else if (strncmp(comm, note_comm, TASK_COMM_LEN) == 0)
		return 1;
	else if (strncmp(comm, hill_climb_comm, TASK_COMM_LEN) == 0)
		return 1;
	else if (strncmp(comm, open_camera_comm, TASK_COMM_LEN) == 0)
		return 1;
	// else if (strncmp(comm, audioserver, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, audio_service_mediatek, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, mediaserver, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, mediaswcodec, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, graphics_service, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, mediatek_service, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, inputmethod, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, camerahalserver, TASK_COMM_LEN) == 0)
	// 	return 1;
	// else if (strncmp(comm, cameraserver, TASK_COMM_LEN) == 0)
	// 	return 1;
	
	return 0;
}

static void print_task_info(struct task_struct *task) 
{
	int pid = task_pid_nr(task);
	unsigned int cpu = task->cpu;

	struct sched_info sched_info = task->sched_info;
	struct sched_entity se = task->se;
	char comm[TASK_COMM_LEN];

	get_task_comm(comm, task);

	pr_warn("[[print_task_info]] %s[%d], CPU: %u, PCOUNT: %lu, RUNTIME: %llu", comm, pid, cpu, sched_info.pcount, se.sum_exec_runtime);
	

	// pr_warn("%s[%d], CPU: %u, SCHED_ENTITY: %llu exec_start, %llu sum_exec_runtime, %llu vruntime", 
	// 		comm, pid, cpu, se.exec_start, se.sum_exec_runtime, se.vruntime);
	
	// pr_warn("SCHED_INFO: %lu times run on CPU, %llu waiting on RQ, %llu last run on CPU", 
	// 		sched_info.pcount, sched_info.run_delay, sched_info.last_arrival);

}

static void print_tasks_cons_info(struct task_cons *tasks, int tasks_n) 
{
	int i;
	struct task_cons t;

	for (i = 0; i < tasks_n; ++i) {
		t = tasks[i];
		pr_warn("[[print_tasks_cons_info]] %s[%d], CPU: %u, PCOUNT: %lu, RUNTIME: %llu", t.comm, t.pid, t.cpu, t.pcount, t.sum_exec_runtime);
	}
}

static struct task_cons task2task_cons(struct task_struct* task)
{
	struct task_cons result;

	struct sched_info sched_info = task->sched_info;
	struct sched_entity se = task->se;

	result.pid = task_pid_nr(task);
	get_task_comm(result.comm, task);

	result.cpu = task->cpu;
	result.pcount = sched_info.pcount;
	result.exec_start = se.exec_start;
	result.sum_exec_runtime = se.sum_exec_runtime;
	result.vruntime = se.vruntime;

	pr_warn("[[task2task_cons]] %s[%d], CPU: %u, PCOUNT: %lu, RUNTIME: %llu", result.comm, result.pid, result.cpu, result.pcount, result.sum_exec_runtime);

	return result;
}

static int get_needed_tasks(struct task_cons *tasks)
{
	struct task_struct *p;

	unsigned tasks_n = 0;
	rcu_read_lock();
    for_each_process(p) {
		if (is_needed_task(p)) {
			// needed_tasks[tasks_n] = task2task_cons(p);
			print_task_info(p);
			tasks[tasks_n] = task2task_cons(p);
			tasks_n += 1;
		}
    }
	rcu_read_unlock();

	return tasks_n;
}


static struct task_cons calculate_task_cons_delta(struct task_cons *before_task_cons, struct task_cons *after_task_cons)
{
	struct task_cons delta;


	delta.pid = after_task_cons->pid;
	strcpy(delta.comm, after_task_cons->comm);
	delta.cpu = after_task_cons->cpu;

	delta.pcount = after_task_cons->pcount - before_task_cons->pcount;
	delta.exec_start = after_task_cons->exec_start - before_task_cons->exec_start;
	delta.sum_exec_runtime = after_task_cons->sum_exec_runtime - before_task_cons->sum_exec_runtime;
	delta.vruntime = after_task_cons->vruntime - before_task_cons->vruntime;

	return delta;
}

static app_type comm2app_type(const char *comm) 
{
	if (strncmp(comm, aimp_comm, TASK_COMM_LEN) == 0)
		return AUDIO;

	if (strncmp(comm, mx_player_comm, TASK_COMM_LEN) == 0)
		return VIDEO;

	if (strncmp(comm, note_comm, TASK_COMM_LEN) == 0)
		return TYPE;

	if (strncmp(comm, hill_climb_comm, TASK_COMM_LEN) == 0)
		return GAME;

	if (strncmp(comm, open_camera_comm, TASK_COMM_LEN) == 0)
		return CAMERA;

	return ANOTHER;
}

static const char *app_type2app_str(app_type app)
{
	switch (app)
	{
	case AUDIO:
		return aimp_comm;
		break;

	case VIDEO:
		return mx_player_comm;
		break;

	case CAMERA:
		return open_camera_comm;
		break;

	case TYPE:
		return note_comm;
		break;

	case GAME:
		return hill_climb_comm;
		break;

	case ANOTHER:
		return "unknown app";
		break;
	default:
		return "unknown app";
		break;
	}
    
}

static app_type detect_app_type(struct task_cons *before_snapshot, struct task_cons *after_snapshot, int before_n, int after_n) 
{
	struct task_cons diff_results[after_n];

	int i, j;
	struct task_cons diff_cons;
	char *max_comm_by_runtime;
	int max_runtime = 0, k = 0, prev_k = k;

	for (i = 0; i < after_n; ++i) {
		int pid = after_snapshot[i].pid;

		prev_k = k;
		for (j = 0; j < before_n; ++j) {
			if (pid == before_snapshot[j].pid) {
				diff_cons = calculate_task_cons_delta(before_snapshot + j, after_snapshot + i);
				diff_results[k] = diff_cons;

				pr_warn("[[DETECT_APP]] difference PID: %d, comm: %s, runtime diff: %llu", diff_cons.pid, diff_cons.comm, diff_cons.sum_exec_runtime);
				k += 1;
				break;
			}
		}

		if (prev_k == k) {
			diff_results[k] = after_snapshot[i];
			k += 1;
		}
	}

	for (i = 0; i < k; ++i) {
		diff_cons = diff_results[i];
		if (diff_cons.sum_exec_runtime > max_runtime) {
			max_runtime = diff_cons.sum_exec_runtime;
			max_comm_by_runtime = diff_cons.comm;
		}
	}

	return comm2app_type(max_comm_by_runtime);
}

static void update_tuners(struct od_dbs_tuners *od_tuners, app_type app) 
{
	int a, b;
	unsigned int tl;

	if (app == VIDEO || app == TYPE || app == GAME) {
		a = 4; b = 1; tl = 90;
	} else if (app == AUDIO) {
		a = 5; b = 1; tl = 90;
	} else if (app == CAMERA) {
		a = 5; b = 4; tl = 85;
	} else {
		a = 2; b = 1; tl = 80;
	}

	od_tuners->alpha = a;
	od_tuners->beta = b;
	od_tuners->target_load = tl;

	pr_warn("+++++++++++++++++++ New a: %d, b: %d, target load: %lu +++++++++++++++++++", a, b, tl);
}

static void working_procedure(struct od_dbs_tuners *od_tuners)
{
	app_type app;
	int before_snap_len, after_snap_len;
	struct task_cons *before_snap;
	struct task_cons *after_snap;

	pr_warn("Entering in working_procedure ...");
	switch (spsa2_sched_staff.phase)
	{
	case 0:
		spsa2_sched_staff.left_snapshot_len = get_needed_tasks(spsa2_sched_staff.left_snapshot);
		spsa2_sched_staff.phase = 1;
		return;
	case 1:
		before_snap_len = spsa2_sched_staff.left_snapshot_len;
		after_snap_len = get_needed_tasks(spsa2_sched_staff.right_snaphot);

		spsa2_sched_staff.right_snapshot_len = after_snap_len;

		before_snap = spsa2_sched_staff.left_snapshot;
		after_snap = spsa2_sched_staff.right_snaphot;

		app = detect_app_type(before_snap, after_snap, before_snap_len, after_snap_len);

		spsa2_sched_staff.phase = 2;
		break;
	case 2:
		before_snap_len = spsa2_sched_staff.right_snapshot_len;
		after_snap_len = get_needed_tasks(spsa2_sched_staff.left_snapshot);

		spsa2_sched_staff.left_snapshot_len = after_snap_len;

		before_snap = spsa2_sched_staff.right_snaphot;
		after_snap = spsa2_sched_staff.left_snapshot;
		
		app = detect_app_type(before_snap, after_snap, before_snap_len, after_snap_len);

		spsa2_sched_staff.phase = 1;
		break;

	default:
		break;
	}

	pr_warn("[[WORKING_PROCEDURE]] Detected app: %s", app_type2app_str(app));
	update_tuners(od_tuners, app);
}


//////////////////////////////////////////////////////////////////
static void dbs_freq_increase(struct cpufreq_policy *policy, unsigned int freq)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;

	if (od_tuners->powersave_bias)
		freq = od_ops.powersave_bias_target(policy, freq,
				CPUFREQ_RELATION_H);
	else if (policy->cur == policy->max)
		return;

	__cpufreq_driver_target(policy, freq, od_tuners->powersave_bias ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

/*
 * Every sampling_rate, we check, if current idle time is less than 20%
 * (default), then we try to increase frequency. Else, we adjust the frequency
 * proportional to load.
 */
static void od_update(struct cpufreq_policy *policy)
{
	app_type app;
	int first_tasks_n, second_tasks_n;
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct od_policy_dbs_info *dbs_info = to_dbs_info(policy_dbs);
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int load = dbs_update(policy);

	unsigned int freq_next, cluster;
	struct spsa2_policy_dbs_info* dbs_info_spsa;
	// struct spsa2_sched_staff* staff;


	// pr_warn("================================= UPD_TIMER: %lu, UPD_COUNTER: %d =================================", upd_timer, upd_counter);
	
	if (spsa2_sched_staff.dvfs_iter_count % 1500 == 0)
		working_procedure(od_tuners);

	spsa2_sched_staff.dvfs_iter_count += 1;
	
	dbs_info->freq_lo = 0;

	cluster = policy->cpu < 6 ? 0 : 1;
	dbs_info_spsa = &(clusters_data[cluster]);

	freq_next = determine_new_freq(policy, dbs_info_spsa, od_tuners, cluster, load);

	__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);

	return;	
}


static unsigned int od_dbs_update(struct cpufreq_policy *policy)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	struct od_policy_dbs_info *dbs_info = to_dbs_info(policy_dbs);
	int sample_type = dbs_info->sample_type;

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = OD_NORMAL_SAMPLE;
	/*
	 * OD_SUB_SAMPLE doesn't make sense if sample_delay_ns is 0, so ignore
	 * it then.
	 */
	if (sample_type == OD_SUB_SAMPLE && policy_dbs->sample_delay_ns > 0) {
		__cpufreq_driver_target(policy, dbs_info->freq_lo,
					CPUFREQ_RELATION_H);
		return dbs_info->freq_lo_delay_us;
	}

	od_update(policy);

	if (dbs_info->freq_lo) {
		/* Setup SUB_SAMPLE */
		dbs_info->sample_type = OD_SUB_SAMPLE;
		return dbs_info->freq_hi_delay_us;
	}

	return dbs_data->sampling_rate * policy_dbs->rate_mult;
}

/************************** sysfs interface ************************/
static struct dbs_governor od_dbs_gov;

static ssize_t store_io_is_busy(struct gov_attr_set *attr_set, const char *buf,
				size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_data->io_is_busy = !!input;

	/* we need to re-evaluate prev_cpu_idle */
	gov_update_cpu_data(dbs_data);

	return count;
}

static ssize_t store_up_threshold(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	dbs_data->up_threshold = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct policy_dbs_info *policy_dbs;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	dbs_data->sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	list_for_each_entry(policy_dbs, &attr_set->policy_list, list) {
		/*
		 * Doing this without locking might lead to using different
		 * rate_mult values in od_update() and od_dbs_update().
		 */
		mutex_lock(&policy_dbs->update_mutex);
		policy_dbs->rate_mult = 1;
		mutex_unlock(&policy_dbs->update_mutex);
	}

	return count;
}

static ssize_t store_ignore_nice_load(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_data->ignore_nice_load) { /* nothing to do */
		return count;
	}
	dbs_data->ignore_nice_load = input;

	/* we need to re-evaluate prev_cpu_idle */
	gov_update_cpu_data(dbs_data);

	return count;
}

static ssize_t store_powersave_bias(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	struct policy_dbs_info *policy_dbs;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	od_tuners->powersave_bias = input;

	list_for_each_entry(policy_dbs, &attr_set->policy_list, list)
		spsa2_sched_powersave_bias_init(policy_dbs->policy);

	return count;
}

static ssize_t store_beta(struct gov_attr_set *attr_set, const char *buf,
		size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
    pr_warn("gov spsa2 got value: %d, store_beta \n", input);
	
    if (ret != 1)
		return -EINVAL;
	od_tuners->beta = input;

	return count;
}

static ssize_t store_alpha(struct gov_attr_set *attr_set, const char *buf,
		size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
    pr_warn("gov spsa2 got value: %d, store_alpha \n", input);
	
    if (ret != 1)
		return -EINVAL;
	od_tuners->alpha = input;

	return count;
}

static ssize_t store_target_load(struct gov_attr_set *attr_set, const char *buf,
		size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
    pr_warn("gov spsa2 got value: %u, store_target_load \n", input);
	
    if (ret != 1)
		return -EINVAL;
	od_tuners->target_load = input;

	return count;
}

static ssize_t store_log_print(struct gov_attr_set *attr_set, const char *buf,
		size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
    pr_warn("gov spsa2 got value: %u, store_log_print \n", input);
	
    if (ret != 1)
		return -EINVAL;
	od_tuners->log_print = !!input;

	return count;
}

gov_show_one_common(sampling_rate);
gov_show_one_common(up_threshold);
gov_show_one_common(sampling_down_factor);
gov_show_one_common(ignore_nice_load);
gov_show_one_common(io_is_busy);
gov_show_one(od, powersave_bias);
gov_show_one(od, beta);
gov_show_one(od, alpha);
gov_show_one(od, target_load);
gov_show_one(od, log_print);


gov_attr_rw(sampling_rate);
gov_attr_rw(io_is_busy);
gov_attr_rw(up_threshold);
gov_attr_rw(sampling_down_factor);
gov_attr_rw(ignore_nice_load);
gov_attr_rw(powersave_bias);
gov_attr_rw(beta);
gov_attr_rw(alpha);
gov_attr_rw(target_load);
gov_attr_rw(log_print);


static struct attribute *od_attributes[] = {
	&sampling_rate.attr,
	&up_threshold.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&beta.attr,
	&alpha.attr,
	&target_load.attr,
	&log_print.attr,
	NULL
};

/************************** sysfs end ************************/

static struct policy_dbs_info *od_alloc(void)
{
	struct od_policy_dbs_info *dbs_info;

	dbs_info = kzalloc(sizeof(*dbs_info), GFP_KERNEL);
	return dbs_info ? &dbs_info->policy_dbs : NULL;
}

static void od_free(struct policy_dbs_info *policy_dbs)
{
	kfree(to_dbs_info(policy_dbs));
}

static int od_init(struct dbs_data *dbs_data)
{
	struct od_dbs_tuners *tuners;
	u64 idle_time;
	int cpu;
	int cluster, freq_amount;
	unsigned int *freq;

	tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
	if (!tuners)
		return -ENOMEM;

	cpu = get_cpu();
	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_data->up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
	} else {
		dbs_data->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
	}

	dbs_data->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
	dbs_data->ignore_nice_load = 0;
	tuners->powersave_bias = default_powersave_bias;
	dbs_data->io_is_busy = should_io_be_busy();

	// spsa2 use
	tuners->alpha = 2;
	tuners->beta = 1;
	tuners->target_load = 80;
	tuners->log_print = 0;
	// spsa2 use end

	dbs_data->tuners = tuners;

	// spsa2 use
	for (cluster = 0; cluster < 2; cluster++)
    {   
        pr_warn("gov spsa2 init spsa2_2 cluster: %d \n", cluster);
        freq = get_freq_array(cluster);    
        freq_amount = get_freq_amount();

        clusters_data[cluster].requested_freq = freq[freq_amount / 2];
        clusters_data[cluster].old_index = freq_amount / 2;
        clusters_data[cluster].delta = 1;
        clusters_data[cluster].direction = 0;
        clusters_data[cluster].phase = 0;
        clusters_data[cluster].old_load = 100;   
    }

	spsa2_sched_staff.dvfs_iter_count = 0;
	spsa2_sched_staff.phase = 0;

	return 0;
}

static void od_exit(struct dbs_data *dbs_data)
{
	kfree(dbs_data->tuners);
}

static void od_start(struct cpufreq_policy *policy)
{
	struct od_policy_dbs_info *dbs_info = to_dbs_info(policy->governor_data);

	dbs_info->sample_type = OD_NORMAL_SAMPLE;
	spsa2_sched_powersave_bias_init(policy);
}

static struct od_ops od_ops = {
	.powersave_bias_target = generic_powersave_bias_target,
};

static struct dbs_governor od_dbs_gov = {
	.gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("spsa2_sched"),
	.kobj_type = { .default_attrs = od_attributes },
	.gov_dbs_update = od_dbs_update,
	.alloc = od_alloc,
	.free = od_free,
	.init = od_init,
	.exit = od_exit,
	.start = od_start,
};

#define CPU_FREQ_GOV_SPSA2_SCHED	(&od_dbs_gov.gov)

static void od_set_powersave_bias(unsigned int powersave_bias)
{
	unsigned int cpu;
	cpumask_t done;

	default_powersave_bias = powersave_bias;
	cpumask_clear(&done);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct policy_dbs_info *policy_dbs;
		struct dbs_data *dbs_data;
		struct od_dbs_tuners *od_tuners;

		if (cpumask_test_cpu(cpu, &done))
			continue;

		policy = cpufreq_cpu_get_raw(cpu);
		if (!policy || policy->governor != CPU_FREQ_GOV_SPSA2_SCHED)
			continue;

		policy_dbs = policy->governor_data;
		if (!policy_dbs)
			continue;

		cpumask_or(&done, &done, policy->cpus);

		dbs_data = policy_dbs->dbs_data;
		od_tuners = dbs_data->tuners;
		od_tuners->powersave_bias = default_powersave_bias;
	}
	put_online_cpus();
}

void od_register_powersave_bias_handler_spsa2_sched(unsigned int (*f)
		(struct cpufreq_policy *, unsigned int, unsigned int),
		unsigned int powersave_bias)
{
	od_ops.powersave_bias_target = f;
	od_set_powersave_bias(powersave_bias);
}
EXPORT_SYMBOL_GPL(od_register_powersave_bias_handler_spsa2_sched);

void od_unregister_powersave_bias_handler_spsa2_sched(void)
{
	od_ops.powersave_bias_target = generic_powersave_bias_target;
	od_set_powersave_bias(0);
}
EXPORT_SYMBOL_GPL(od_unregister_powersave_bias_handler_spsa2_sched);

static int __init cpufreq_gov_dbs_init(void)
{
	return cpufreq_register_governor(CPU_FREQ_GOV_SPSA2_SCHED);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(CPU_FREQ_GOV_SPSA2_SCHED);
}

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return CPU_FREQ_GOV_SPSA2_SCHED;
}

fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
