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
            PROFILER(FRAME) \
            PROFILER(ENCODE_FRAME) \
            PROFILER(ENCODER_FRAME_END) \
            PROFILER(FRAME_COPY_PICTURE) \
            PROFILER(FRAME_SORT) \
            PROFILER(REFERENCE_BUILD_LIST) \
            PROFILER(SPS_WRITE) \
            PROFILER(PPS_WRITE) \
            PROFILER(NUM_PROFILERS)

#define PROFILER_GENERATE_ENUM(ENUM) ENUM,
 
typedef enum {
    PROFILER_FOREACH(PROFILER_GENERATE_ENUM)
} PROFILER;

#endif
