//#####################################################################
// Copyright 2004, Igor Neverov, Eftychios Sifakis.
// This file is part of PhysBAM whose distribution is governed by the license contained in the accompanying file PHYSBAM_COPYRIGHT.txt.
//#####################################################################
#include "../../Public_Library/Utilities/PARSE_ARGS.h"
#include "../../Public_Library/Thread_Utilities/THREAD_POOL.h"
#include "../../Public_Library/Thread_Utilities/THREAD_DIVISION_PARAMETERS.h"

#include "FACE_DRIVER.h"
#include "Storytelling/STORYTELLING_EXAMPLE.h"
#include "../../Public_Library/Utilities/LOG.h"

#ifdef ENABLE_PARSEC_HOOKS
#include <hooks.h>
#endif

#include <heartbeats/heartbeat-accuracy-power.h>
#include <poet.h>
#include <poet_config.h>

#define PREFIX "FACESIM"
#define USE_POET // Power and performance control

heartbeat_t* heart;
poet_state* s_state;
static poet_control_state_t* s_control_states;
static poet_cpu_state_t* s_cpu_states;
int perf_pwr_switch;

using namespace PhysBAM;

#ifdef ENABLE_PTHREADS
//Use serial code
bool PHYSBAM_THREADED_RUN = true;
# else
//Use multi-threaded code
bool PHYSBAM_THREADED_RUN = false;
#endif //ENABLE_PTHREADS

static inline void hb_poet_init() {
    float min_heartrate;
    float max_heartrate;
    int window_size;
    double power_target;
    unsigned int s_nstates;
    poet_tradeoff_type_t constraint;
    double constraint_goal;

    if(getenv(PREFIX"_MIN_HEART_RATE") == NULL) {
      min_heartrate = 0.0;
    } else {
      min_heartrate = atof(getenv(PREFIX"_MIN_HEART_RATE"));
    }
    if(getenv(PREFIX"_MAX_HEART_RATE") == NULL) {
      max_heartrate = 100.0;
    } else {
      max_heartrate = atof(getenv(PREFIX"_MAX_HEART_RATE"));
    }
    if(getenv(PREFIX"_WINDOW_SIZE") == NULL) {
      window_size = 20;
    } else {
      window_size = atoi(getenv(PREFIX"_WINDOW_SIZE"));
    }
    if(getenv(PREFIX"_POWER_TARGET") == NULL) {
      power_target = 100;
    } else {
      power_target = atof(getenv(PREFIX"_POWER_TARGET"));
    }
    if(getenv(PREFIX"_CONSTRAINT") == NULL) {
      constraint = PERFORMANCE;
      constraint_goal = (min_heartrate + max_heartrate) / 2.0;
    } else if (strcmp(getenv(PREFIX"_CONSTRAINT"), "POWER") == 0){
      constraint = POWER;
      constraint_goal = power_target;
    } else {
      constraint = PERFORMANCE;
      constraint_goal = (min_heartrate + max_heartrate) / 2.0;
    }
    if (getenv(PREFIX"_PERF_PWR_SWITCH_HB") == NULL) {
      perf_pwr_switch = -1;
    } else {
      perf_pwr_switch = atoi(getenv(PREFIX"_PERF_PWR_SWITCH_HB"));
    }

    printf("init heartbeat with %f %f %f %d\n", min_heartrate, max_heartrate, power_target, window_size);
    heart = heartbeat_acc_pow_init(window_size, 100, "heartbeat.log",
                                   min_heartrate, max_heartrate,
                                   0, 100,
                                   power_target, power_target);
    if (heart == NULL) {
      fprintf(stderr, "Failed to init heartbeat.\n");
      exit(1);
    }
#ifdef USE_POET
    if (get_control_states(NULL, &s_control_states, &s_nstates)) {
      fprintf(stderr, "Failed to load control states.\n");
      exit(1);
    }
    if (get_cpu_states(NULL, &s_cpu_states, &s_nstates)) {
      fprintf(stderr, "Failed to load cpu states.\n");
      exit(1);
    }
    s_state = poet_init(constraint_goal, constraint, s_nstates, s_control_states, s_cpu_states, &apply_cpu_config, &get_current_cpu_state, window_size, 1, "poet.log");
    if (s_state == NULL) {
      fprintf(stderr, "Failed to init poet.\n");
      exit(1);
    }
#endif
   printf("heartbeat init'd\n");

}

static inline void hb_poet_finish() {
#ifdef USE_POET
    poet_destroy(s_state);
    free(s_control_states);
    free(s_cpu_states);
#endif
    heartbeat_finish(heart);
    printf("heartbeat finished\n");
}

int main (int argc, char* argv[])
{
#ifdef PARSEC_VERSION
#define __PARSEC_STRING(x) #x
#define __PARSEC_XSTRING(x) __PARSEC_STRING(x)
	printf ("PARSEC Benchmark Suite Version "__PARSEC_XSTRING (PARSEC_VERSION) "\n");
	fflush (NULL);
#else
	printf ("PARSEC Benchmark Suite\n");
	fflush (NULL);
#endif //PARSEC_VERSION
#ifdef ENABLE_PARSEC_HOOKS
	__parsec_bench_begin (__parsec_facesim);
#endif

	PARSE_ARGS parse_args;
	parse_args.Add_Integer_Argument ("-restart", 0);
	parse_args.Add_Integer_Argument ("-lastframe", 300);
	parse_args.Add_Integer_Argument ("-threads", 1);
	parse_args.Add_Option_Argument ("-timing");
	parse_args.Parse (argc, argv);

	STORYTELLING_EXAMPLE<float, float> example;

	FILE_UTILITIES::Create_Directory (example.output_directory);
	LOG::Copy_Log_To_File (example.output_directory + "/log.txt", example.restart);

	if (parse_args.Is_Value_Set ("-threads"))
	{
		static char tmp_buf[255];
		sprintf (tmp_buf, "PHYSBAM_THREADS=%d", parse_args.Get_Integer_Value ("-threads"));

		if (putenv (tmp_buf) < 0) perror ("putenv");
	}

	if (parse_args.Is_Value_Set ("-restart"))
	{
		example.restart = true;
		example.restart_frame = parse_args.Get_Integer_Value ("-restart");
	}

	if (parse_args.Is_Value_Set ("-lastframe"))
	{
		example.last_frame = parse_args.Get_Integer_Value ("-lastframe");
	}

	if (parse_args.Is_Value_Set ("-timing"))
	{
		example.write_output_files = false;
		example.verbose = false;
	}

	if (PHYSBAM_THREADED_RUN == false && parse_args.Get_Integer_Value ("-threads") > 1)
	{
		printf ("Error: Number of threads cannot be greater than 1 for serial runs\n");
		exit (1);
	}

	hb_poet_init();

	THREAD_DIVISION_PARAMETERS<float>& parameters = *THREAD_DIVISION_PARAMETERS<float>::Singleton();
	parameters.grid_divisions_3d = VECTOR_3D<int> (5, 5, 5);

	FACE_DRIVER<float, float> driver (example);

	driver.Execute_Main_Program();

	delete (THREAD_POOL::Singleton());

	hb_poet_finish();

#ifdef ENABLE_PARSEC_HOOKS
	__parsec_bench_end();
#endif

	return 0;
}
