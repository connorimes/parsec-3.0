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

#define HB_ENERGY_IMPL
#include <heartbeats/hb-energy.h>
#include <heartbeats/heartbeat-accuracy-power.h>
#include <raplcap.h>
#include <copper.h>
#include <copper-util.h>

#define PREFIX "FACESIM"

heartbeat_t* heart;
raplcap rc;
copper* cop;

using namespace PhysBAM;

#ifdef ENABLE_PTHREADS
//Use serial code
bool PHYSBAM_THREADED_RUN = true;
# else
//Use multi-threaded code
bool PHYSBAM_THREADED_RUN = false;
#endif //ENABLE_PTHREADS

int apply_powercap(double powercap) {
  if (getenv("POWERCAP_DISABLE") != NULL) {
    return 0;
  }
  raplcap_limit rl;
  uint32_t i;
  uint32_t n = raplcap_get_num_sockets(&rc);
  if (n == 0) {
    perror("raplcap_get_num_sockets");
    return -1;
  }
  printf("Requested power cap %f for %"PRIu32" sockets\n", powercap, n);
  // share powercap evenly across sockets
  // time window of zero keeps current time window
  rl.seconds = 0.0;
  rl.watts = powercap / (double) n;
  for (i = 0; i < n; i++) {
    printf("New RAPL config for socket %"PRIu32": time=%f power=%f\n", i, rl.seconds, rl.watts);
    if (raplcap_set_limits(i, &rc, RAPLCAP_ZONE_PACKAGE, NULL, &rl)) {
      perror("raplcap_set_limits");
      return -1;
    }
  }
  return 0;
}

static inline void hb_copper_init() {
  double min_heartrate = 0.0;
  double max_heartrate = 100.0;
  int window_size = 20;
  double min_power = 0.1;
  double max_power = 100.0;
  const char* model = NULL;

  if (getenv(PREFIX"_MIN_HEART_RATE") != NULL) {
    min_heartrate = atof(getenv(PREFIX"_MIN_HEART_RATE"));
  }
  if (getenv(PREFIX"_MAX_HEART_RATE") != NULL) {
    max_heartrate = atof(getenv(PREFIX"_MAX_HEART_RATE"));
  }
  if (getenv(PREFIX"_WINDOW_SIZE") != NULL) {
    window_size = atoi(getenv(PREFIX"_WINDOW_SIZE"));
  }
  if (getenv(PREFIX"_MIN_POWER") != NULL) {
    min_power = atof(getenv(PREFIX"_MIN_POWER"));
  }
  if (getenv(PREFIX"_MAX_POWER") != NULL) {
    max_power = atof(getenv(PREFIX"_MAX_POWER"));
  }
  model = getenv(PREFIX"_MODEL");

  printf("init heartbeat with %f %f %f %f %d\n", min_heartrate, max_heartrate, min_power, max_power, window_size);
  heart = heartbeat_acc_pow_init(window_size, 100, "heartbeat.log",
                                 min_heartrate, max_heartrate,
                                 0, 100,
                                 1, hb_energy_impl_alloc(), min_power, max_power);
  if (heart == NULL) {
    fprintf(stderr, "Failed to init heartbeat.\n");
    exit(1);
  }
  printf("heartbeat init'd\n");
  if (raplcap_init(&rc)) {
    perror("raplcap_init");
    exit(1);
  }
  // start at max power
  if (apply_powercap(max_power)) {
    perror("apply_powercap");
    exit(1);
  }
  printf("raplcap init'd\n");
  cop = copper_alloc_init((min_heartrate + max_heartrate) / 2.0, min_power, max_power, max_power, 1, "copper.log", model);
  if (cop == NULL) {
    perror("copper_alloc_init");
    exit(1);
  }
  printf("copper init'd\n");
}

static inline void hb_copper_finish() {
  copper_destroy_free(cop);
  printf("copper destroyed\n");
  raplcap_destroy(&rc);
  printf("raplcap destroyed\n");
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

	hb_copper_init();

	THREAD_DIVISION_PARAMETERS<float>& parameters = *THREAD_DIVISION_PARAMETERS<float>::Singleton();
	parameters.grid_divisions_3d = VECTOR_3D<int> (5, 5, 5);

	FACE_DRIVER<float, float> driver (example);

	driver.Execute_Main_Program();

	delete (THREAD_POOL::Singleton());

	hb_copper_finish();

#ifdef ENABLE_PARSEC_HOOKS
	__parsec_bench_end();
#endif

	return 0;
}
