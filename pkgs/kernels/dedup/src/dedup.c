#include <unistd.h>
#include <string.h>

#include "util.h"
#include "debug.h"
#include "dedupdef.h"
#include "encoder.h"
#include "decoder.h"
#include "config.h"
#include "queue.h"

#ifdef ENABLE_DMALLOC
#include <dmalloc.h>
#endif //ENABLE_DMALLOC

#ifdef ENABLE_PTHREADS
#include <pthread.h>
#endif //ENABLE_PTHREADS

#ifdef ENABLE_PARSEC_HOOKS
#include <hooks.h>
#endif //ENABLE_PARSEC_HOOKS

#define HB_ENERGY_IMPL
#include <heartbeats/hb-energy.h>
#include <heartbeats/heartbeat-accuracy-power.h>
#include <poet/poet.h>
#include <poet/poet_config.h>

#define PREFIX "DEDUP"
#define USE_POET // Power and performance control

heartbeat_t* heart;
poet_state* s_state;
static poet_control_state_t* s_control_states;
static poet_cpu_state_t* s_cpu_states;

config_t * conf;

static inline void hb_poet_init() {
    float min_heartrate;
    float max_heartrate;
    int window_size;
    double power_target;
    unsigned int s_nstates;

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
      window_size = 30;
    } else {
      window_size = atoi(getenv(PREFIX"_WINDOW_SIZE"));
    }
    if(getenv(PREFIX"_POWER_TARGET") == NULL) {
      power_target = 70;
    } else {
      power_target = atof(getenv(PREFIX"_POWER_TARGET"));
    }

    printf("init heartbeat with %f %f %d\n", min_heartrate, max_heartrate, window_size);
    heart = heartbeat_acc_pow_init(window_size, 100, "heartbeat.log",
                                   min_heartrate, max_heartrate,
                                   0, 100,
                                   1, hb_energy_impl_alloc(), power_target, power_target);
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
    s_state = poet_init(heart, s_nstates, s_control_states, s_cpu_states, &apply_cpu_config, &get_current_cpu_state, 1, "poet.log");
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

/*--------------------------------------------------------------------------*/
static void
usage(char* prog)
{
  printf("usage: %s [-cusfvh] [-w gzip/bzip2/none] [-i file] [-o file] [-t number_of_threads]\n",prog);
  printf("-c \t\t\tcompress\n");
  printf("-u \t\t\tuncompress\n");
  printf("-p \t\t\tpreloading (for benchmarking purposes)\n");
  printf("-w \t\t\tcompression type: gzip/bzip2/none\n");
  printf("-i file\t\t\tthe input file\n");
  printf("-o file\t\t\tthe output file\n");
  printf("-t \t\t\tnumber of threads per stage \n");
  printf("-v \t\t\tverbose output\n");
  printf("-h \t\t\thelp\n");
}
/*--------------------------------------------------------------------------*/
int main(int argc, char** argv) {
  hb_poet_init();

#ifdef PARSEC_VERSION
#define __PARSEC_STRING(x) #x
#define __PARSEC_XSTRING(x) __PARSEC_STRING(x)
  printf("PARSEC Benchmark Suite Version "__PARSEC_XSTRING(PARSEC_VERSION)"\n");
#else
  printf("PARSEC Benchmark Suite\n");
#endif //PARSEC_VERSION
#ifdef ENABLE_PARSEC_HOOKS
        __parsec_bench_begin(__parsec_dedup);
#endif //ENABLE_PARSEC_HOOKS

  int32 compress = TRUE;

  //We force the sha1 sum to be integer-aligned, check that the length of a sha1 sum is a multiple of unsigned int
  assert(SHA1_LEN % sizeof(unsigned int) == 0);

  conf = (config_t *) malloc(sizeof(config_t));
  if (conf == NULL) {
    EXIT_TRACE("Memory allocation failed\n");
  }

  strcpy(conf->outfile, "");
  conf->compress_type = COMPRESS_GZIP;
  conf->preloading = 0;
  conf->nthreads = 1;
  conf->verbose = 0;

  //parse the args
  int ch;
  opterr = 0;
  optind = 1;
  while (-1 != (ch = getopt(argc, argv, "cupvo:i:w:t:h"))) {
    switch (ch) {
    case 'c':
      compress = TRUE;
      strcpy(conf->infile, "test.txt");
      strcpy(conf->outfile, "out.ddp");
      break;
    case 'u':
      compress = FALSE;
      strcpy(conf->infile, "out.ddp");
      strcpy(conf->outfile, "new.txt");
      break;
    case 'w':
      if (strcmp(optarg, "gzip") == 0)
        conf->compress_type = COMPRESS_GZIP;
      else if (strcmp(optarg, "bzip2") == 0) 
        conf->compress_type = COMPRESS_BZIP2;
      else if (strcmp(optarg, "none") == 0)
        conf->compress_type = COMPRESS_NONE;
      else {
        fprintf(stdout, "Unknown compression type `%s'.\n", optarg);
        usage(argv[0]);
        return -1;
      }
      break;
    case 'o':
      strcpy(conf->outfile, optarg);
      break;
    case 'i':
      strcpy(conf->infile, optarg);
      break;
    case 'h':
      usage(argv[0]);
      hb_poet_finish();
      return -1;
    case 'p':
      conf->preloading = TRUE;
      break;
    case 't':
      conf->nthreads = atoi(optarg);
      break;
    case 'v':
      conf->verbose = TRUE;
      break;
    case '?':
      fprintf(stdout, "Unknown option `-%c'.\n", optopt);
      usage(argv[0]);
      hb_poet_finish();
      return -1;
    }
  }

#ifndef ENABLE_BZIP2_COMPRESSION
 if (conf->compress_type == COMPRESS_BZIP2){
    printf("Bzip2 compression not supported\n");
    hb_poet_finish();
    exit(1);
  }
#endif

#ifndef ENABLE_GZIP_COMPRESSION
 if (conf->compress_type == COMPRESS_GZIP){
    printf("Gzip compression not supported\n");
    hb_poet_finish();
    exit(1);
  }
#endif

#ifndef ENABLE_STATISTICS
 if (conf->verbose){
    printf("Statistics collection not supported\n");
    exit(1);
  }
#endif

#ifndef ENABLE_PTHREADS
 if (conf->nthreads != 1){
    printf("Number of threads must be 1 (serial version)\n");
    exit(1);
  }
#endif

  if (compress) {
    Encode(conf);
  } else {
    Decode(conf);
  }

  free(conf);

#ifdef ENABLE_PARSEC_HOOKS
  __parsec_bench_end();
#endif

  hb_poet_finish();

  return 0;
}

