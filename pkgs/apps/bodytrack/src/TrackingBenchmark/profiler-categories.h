/**
 * Profiling categories.
 *
 * @author Connor Imes
 * @date 2015-11-11
 */

#ifndef PROFILER_CATEGORIES_H
#define PROFILER_CATEGORIES_H

// generate profiler category enums and names
#define PROFILER_FOREACH(PROFILER) \
            PROFILER(APPLICATION) \
            PROFILER(PARTICLE_FILTER_UPDATE) \
            PROFILER(PARTICLE_FILTER_ESTIMATE) \
            PROFILER(WRITE_POSE) \
            PROFILER(OUTPUT_BMP) \
            PROFILER(NUM_PROFILERS)

#define PROFILER_GENERATE_ENUM(ENUM) ENUM,
 
typedef enum {
    PROFILER_FOREACH(PROFILER_GENERATE_ENUM)
} PROFILER;

#endif
