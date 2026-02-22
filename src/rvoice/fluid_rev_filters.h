/*
 * FluidSynth - Dedicated reverb filter building blocks
 *
 * Provides C++11 classes for allpass filters, comb filters, and delay lines
 * used by the reverb engines. The classes are header-only to allow easy reuse
 * across the different reverbs without changing their behavior.
 */

#pragma once

#ifndef FLUID_REV_FILTERS_H
#define FLUID_REV_FILTERS_H

#include "fluid_sys.h"

/** Algorithm variant used by the allpass filter. */
enum fluid_reverb_allpass_mode
{
    /** Freeverb-style allpass implementation. */
    FLUID_REVERB_ALLPASS_FREEVERB,
    /** Standard Schroeder allpass implementation. */
    FLUID_REVERB_ALLPASS_SCHROEDER
};

/**
 * @brief Allpass filter stage for reverb processing.
 *
 * @tparam SampleType Floating point sample type (float or double).
 */
template<typename SampleType>
class fluid_reverb_allpass
{
public:
    /** Set which algorithm variant to use when processing samples. */
    void set_mode(fluid_reverb_allpass_mode mode_in)
    {
        mode = mode_in;
    }

    /** Set the feedback coefficient controlling the allpass response. */
    void set_feedback(SampleType value)
    {
        feedback = value;
    }

    /** Get the feedback coefficient. */
    SampleType get_feedback() const
    {
        return feedback;
    }

    /** Allocate the delay buffer with the given length. */
    bool set_buffer(int size)
    {
        buffer = FLUID_ARRAY(SampleType, size);
        if(buffer == NULL)
        {
            buffer_size = 0;
            return false;
        }
        buffer_size = size;
        buffer_index = 0;
        last_output = 0;
        return true;
    }

    /** Release the delay buffer and reset state. */
    void release()
    {
        FLUID_FREE(buffer);
        buffer = NULL;
        buffer_size = 0;
        buffer_index = 0;
        last_output = 0;
    }

    /** Fill the delay buffer without changing the current index. */
    void fill_buffer(SampleType value)
    {
        for(int i = 0; i < buffer_size; ++i)
        {
            buffer[i] = value;
        }
    }

    /** Set the current delay buffer index (used when resetting state). */
    void set_index(int index)
    {
        buffer_index = index;
    }

    /** Set the cached output value (used for lexverb cross-feedback). */
    void set_last_output(SampleType value)
    {
        last_output = value;
    }

    /** Return the most recently produced output sample. */
    SampleType get_last_output() const
    {
        return last_output;
    }

    /** Check if a buffer has been allocated. */
    bool has_buffer() const
    {
        return buffer != NULL;
    }

    /**
     * Process a single sample through the allpass filter.
     *
     * @param input Input sample.
     * @return Filtered output sample.
     */
    SampleType process(SampleType input)
    {
        SampleType bufout = buffer[buffer_index];
        SampleType output;

        if(mode == FLUID_REVERB_ALLPASS_FREEVERB)
        {
            output = bufout - input;
            buffer[buffer_index] = input + (bufout * feedback);
        }
        else
        {
            SampleType delay_in = input + (bufout * feedback);
            output = bufout - (delay_in * feedback);
            buffer[buffer_index] = delay_in;
        }

        if(++buffer_index >= buffer_size)
        {
            buffer_index = 0;
        }

        last_output = output;
        return output;
    }

    /** Algorithm variant selector. */
    fluid_reverb_allpass_mode mode;
    /** Feedback coefficient (g) for the allpass filter. */
    SampleType feedback;
    /** Delay buffer storage for the filter. */
    SampleType *buffer;
    /** Length of the delay buffer in samples. */
    int buffer_size;
    /** Current index into the delay buffer. */
    int buffer_index;
    /** Last output sample produced by process(). */
    SampleType last_output;
};

/**
 * @brief Comb filter stage for reverb processing.
 *
 * @tparam SampleType Floating point sample type (float or double).
 */
template<typename SampleType>
class fluid_reverb_comb
{
public:
    /** Allocate the delay buffer with the given length. */
    bool set_buffer(int size)
    {
        buffer = FLUID_ARRAY(SampleType, size);
        if(buffer == NULL)
        {
            buffer_size = 0;
            return false;
        }
        buffer_size = size;
        buffer_index = 0;
        filterstore = 0;
        return true;
    }

    /** Release the delay buffer and reset state. */
    void release()
    {
        FLUID_FREE(buffer);
        buffer = NULL;
        buffer_size = 0;
        buffer_index = 0;
    }

    /** Fill the delay buffer without changing the current index. */
    void fill_buffer(SampleType value)
    {
        for(int i = 0; i < buffer_size; ++i)
        {
            buffer[i] = value;
        }
    }

    /** Set the damping value (0..1) which controls the comb low pass. */
    void set_damp(SampleType value)
    {
        damp1 = value;
        damp2 = (SampleType)1 - value;
    }

    /** Return the current damping value. */
    SampleType get_damp() const
    {
        return damp1;
    }

    /** Set the feedback coefficient for the comb filter. */
    void set_feedback(SampleType value)
    {
        feedback = value;
    }

    /** Return the feedback coefficient. */
    SampleType get_feedback() const
    {
        return feedback;
    }

    /** Check if a buffer has been allocated. */
    bool has_buffer() const
    {
        return buffer != NULL;
    }

    /**
     * Process a single sample through the comb filter.
     *
     * @param input Input sample.
     * @return Filtered output sample.
     */
    SampleType process(SampleType input)
    {
        SampleType output = buffer[buffer_index];
        filterstore = (output * damp2) + (filterstore * damp1);
        buffer[buffer_index] = input + (filterstore * feedback);

        if(++buffer_index >= buffer_size)
        {
            buffer_index = 0;
        }

        return output;
    }

    /** Feedback coefficient (roomsize-dependent). */
    SampleType feedback;
    /** Internal low-pass filter storage. */
    SampleType filterstore;
    /** Damping coefficient (damp1) for the low-pass filter. */
    SampleType damp1;
    /** Complementary damping coefficient (damp2). */
    SampleType damp2;
    /** Delay buffer storage for the filter. */
    SampleType *buffer;
    /** Length of the delay buffer in samples. */
    int buffer_size;
    /** Current index into the delay buffer. */
    int buffer_index;
};

/**
 * @brief Damping low-pass filter state for delay lines.
 *
 * @tparam SampleType Floating point sample type (float or double).
 */
template<typename SampleType>
struct fluid_reverb_delay_damping
{
    /** Filter history value. */
    SampleType buffer;
    /** Feed-forward coefficient. */
    SampleType b0;
    /** Feedback coefficient. */
    SampleType a1;
};

/**
 * @brief Delay line used by reverb algorithms.
 *
 * @tparam SampleType Floating point sample type (float or double).
 */
template<typename SampleType, typename DampingType = fluid_reverb_delay_damping<SampleType> >
class fluid_reverb_delay_line
{
public:
    /** Allocate the delay buffer with the given length. */
    bool set_buffer(int length)
    {
        line = FLUID_ARRAY(SampleType, length);
        if(line == NULL)
        {
            size = 0;
            return false;
        }
        size = length;
        line_in = 0;
        line_out = 0;
        last_output = 0;
        return true;
    }

    /** Release the delay buffer and reset indices. */
    void release()
    {
        FLUID_FREE(line);
        line = NULL;
        size = 0;
        line_in = 0;
        line_out = 0;
        last_output = 0;
    }

    /** Fill the delay buffer without changing indices. */
    void fill_buffer(SampleType value)
    {
        for(int i = 0; i < size; ++i)
        {
            line[i] = value;
        }
    }

    /** Set the current read/write indices. */
    void set_positions(int in_pos, int out_pos)
    {
        line_in = in_pos;
        line_out = out_pos;
    }

    /** Set the coefficient used by lexverb delay mixing. */
    void set_coefficient(SampleType value)
    {
        coefficient = value;
    }

    /** Return the coefficient used by lexverb delay mixing. */
    SampleType get_coefficient() const
    {
        return coefficient;
    }

    /** Return the most recently produced output sample. */
    SampleType get_last_output() const
    {
        return last_output;
    }

    /** Check if a buffer has been allocated. */
    bool has_buffer() const
    {
        return line != NULL;
    }

    /**
     * Process a single sample through the delay line (read/write same position).
     *
     * @param input Input sample.
     * @return Delayed output sample.
     */
    SampleType process(SampleType input)
    {
        SampleType output = line[line_out];
        line[line_out] = input;

        if(++line_out >= size)
        {
            line_out = 0;
        }

        line_in = line_out;
        last_output = output;
        return output;
    }

    /** Delay buffer storage. */
    SampleType *line;
    /** Length of the delay buffer in samples. */
    int size;
    /** Write index into the delay buffer. */
    int line_in;
    /** Read index into the delay buffer. */
    int line_out;
    /** Optional damping low-pass filter state. */
    DampingType damping;
    /** Optional coefficient for lexverb cross-feed. */
    SampleType coefficient;
    /** Last output sample produced by process(). */
    SampleType last_output;
};

#endif
