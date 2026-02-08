/******************************************************************************
 * FluidSynth - A Software Synthesizer
 *
 * Copyright (C) 2003  Peter Hanappe and others.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <https://www.gnu.org/licenses/>.
 */

#include "fluid_rev.h"
#include "fluid_sys.h"

extern "C" {
#include "allpass.h"
#include "delay.h"
}

#include "LEXverb.h"

#include <new>

namespace
{
constexpr float LEX_TRIM = 0.7f;
constexpr float LEX_SCALE_WET_WIDTH = 0.2f;
}

struct fluid_revmodel_lexverb : public _fluid_revmodel_t
{
    fluid_real_t roomsize;
    fluid_real_t damp;
    fluid_real_t level;
    fluid_real_t wet1;
    fluid_real_t wet2;
    fluid_real_t width;
    bool valid;
    float damp_state_left;
    float damp_state_right;

    ap_info ap[NUM_OF_AP_SECTS];
    dl_info dl[NUM_OF_DELAY_SECTS];

    explicit fluid_revmodel_lexverb(fluid_real_t sample_rate);
    ~fluid_revmodel_lexverb() override;

    bool is_valid() const
    {
        return valid;
    }

    void processmix(const fluid_real_t *in, fluid_real_t *left_out,
                    fluid_real_t *right_out) override;
    void processreplace(const fluid_real_t *in, fluid_real_t *left_out,
                        fluid_real_t *right_out) override;
    void reset() override;
    void set(int set, fluid_real_t roomsize, fluid_real_t damping,
             fluid_real_t width, fluid_real_t level) override;
    int samplerate_change(fluid_real_t sample_rate) override;
};

typedef struct fluid_revmodel_lexverb fluid_revmodel_lexverb_t;

static void fluid_lexverb_release_blocks(fluid_revmodel_lexverb_t *rev)
{
    int i;

    for(i = 0; i < NUM_OF_AP_SECTS; ++i)
    {
        FLUID_FREE(rev->ap[i].base);
        rev->ap[i].base = NULL;
    }

    for(i = 0; i < NUM_OF_DELAY_SECTS; ++i)
    {
        FLUID_FREE(rev->dl[i].base);
        rev->dl[i].base = NULL;
    }
}

static void fluid_lexverb_clear_blocks(fluid_revmodel_lexverb_t *rev)
{
    int i;

    for(i = 0; i < NUM_OF_AP_SECTS; ++i)
    {
        if(rev->ap[i].base != NULL)
        {
            FLUID_MEMSET(rev->ap[i].base, 0, sizeof(float) * rev->ap[i].length);
            rev->ap[i].out_ptr = rev->ap[i].base + 1;
        }
        rev->ap[i].in_data = 0.0f;
        rev->ap[i].out_data = 0.0f;
    }

    for(i = 0; i < NUM_OF_DELAY_SECTS; ++i)
    {
        if(rev->dl[i].base != NULL)
        {
            FLUID_MEMSET(rev->dl[i].base, 0, sizeof(float) * rev->dl[i].length);
            rev->dl[i].out_ptr = rev->dl[i].base + 1;
        }
        rev->dl[i].in_data = 0.0f;
        rev->dl[i].out_data = 0.0f;
    }

    rev->damp_state_left = 0.0f;
    rev->damp_state_right = 0.0f;
}

static int fluid_lexverb_ms_to_buf_length(float ms, fluid_real_t sample_rate)
{
    return ms * (sample_rate * (1 / 1000.0f));
}

static int fluid_lexverb_setup_blocks(fluid_revmodel_lexverb_t *rev, fluid_real_t sample_rate)
{
    int i;
    for(i = 0; i < NUM_OF_AP_SECTS; ++i)
    {
        int length = fluid_lexverb_ms_to_buf_length(LEX_REVERB_PARMS[i].length, sample_rate);

        rev->ap[i].length = length;
        rev->ap[i].coef = LEX_REVERB_PARMS[i].coef;
        rev->ap[i].base = FLUID_ARRAY(float, length);
        if(rev->ap[i].base == NULL)
        {
            return FLUID_FAILED;
        }
        rev->ap[i].out_ptr = rev->ap[i].base + 1;
        rev->ap[i].in_data = 0.0f;
        rev->ap[i].out_data = 0.0f;
    }

    for(i = 0; i < NUM_OF_DELAY_SECTS; ++i)
    {
        int index = NUM_OF_AP_SECTS + i;
        int length = fluid_lexverb_ms_to_buf_length(LEX_REVERB_PARMS[index].length, sample_rate);

        rev->dl[i].length = length;
        rev->dl[i].coef = LEX_REVERB_PARMS[index].coef;
        rev->dl[i].base = FLUID_ARRAY(float, length);
        if(rev->dl[i].base == NULL)
        {
            return FLUID_FAILED;
        }
        rev->dl[i].out_ptr = rev->dl[i].base + 1;
        rev->dl[i].in_data = 0.0f;
        rev->dl[i].out_data = 0.0f;
    }

    fluid_lexverb_clear_blocks(rev);

    return FLUID_OK;
}

static void fluid_lexverb_update(fluid_revmodel_lexverb_t *rev)
{
    fluid_real_t roomscale = 0.5f + 0.5f * rev->roomsize;
    fluid_real_t wet = (rev->level * roomscale) /
                       (1.0f + rev->width * LEX_SCALE_WET_WIDTH);

    rev->wet1 = wet * (rev->width / 2.0f + 0.5f);
    rev->wet2 = wet * ((1.0f - rev->width) / 2.0f);
}

static void fluid_lexverb_process_sample(fluid_revmodel_lexverb_t *rev, float input,
                                         float *out_left, float *out_right)
{
    ap_info *ap = rev->ap;
    dl_info *dl = rev->dl;

    ap[0].in_data = input * LEX_TRIM;
    ap[1].in_data = all_pass_filter(&ap[0]);
    dl[1].in_data = ap[9].out_data;
    ap[2].in_data = all_pass_filter(&ap[1]) + delay(&dl[1]) * dl[1].coef;
    ap[3].in_data = all_pass_filter(&ap[2]);
    ap[4].in_data = all_pass_filter(&ap[3]);
    *out_left = all_pass_filter(&ap[4]);

    ap[5].in_data = input * LEX_TRIM;
    ap[6].in_data = all_pass_filter(&ap[5]);
    dl[0].in_data = ap[4].out_data;
    ap[7].in_data = all_pass_filter(&ap[6]) + delay(&dl[0]) * dl[0].coef;
    ap[8].in_data = all_pass_filter(&ap[7]);
    ap[9].in_data = all_pass_filter(&ap[8]);
    *out_right = all_pass_filter(&ap[9]);

    if(rev->damp > 0.0f)
    {
        float damp = (float)rev->damp;
        *out_left = *out_left * (1.0f - damp) + rev->damp_state_left * damp;
        *out_right = *out_right * (1.0f - damp) + rev->damp_state_right * damp;
    }

    rev->damp_state_left = *out_left;
    rev->damp_state_right = *out_right;
}

fluid_revmodel_lexverb::fluid_revmodel_lexverb(fluid_real_t sample_rate)
    : roomsize(0.0f),
      damp(0.0f),
      level(0.0f),
      wet1(0.0f),
      wet2(0.0f),
      width(0.0f),
      valid(false),
      damp_state_left(0.0f),
      damp_state_right(0.0f)
{
    if(sample_rate <= 0.0f)
    {
        return;
    }

    FLUID_MEMSET(ap, 0, sizeof(ap));
    FLUID_MEMSET(dl, 0, sizeof(dl));

    if(fluid_lexverb_setup_blocks(this, sample_rate) != FLUID_OK)
    {
        fluid_lexverb_release_blocks(this);
        FLUID_LOG(FLUID_ERR, "LEXverb reverb: failed to allocate delay lines");
        return;
    }

    valid = true;
}

fluid_revmodel_lexverb::~fluid_revmodel_lexverb()
{
    fluid_lexverb_release_blocks(this);
}

void fluid_revmodel_lexverb::processmix(const fluid_real_t *in, fluid_real_t *left_out,
                                        fluid_real_t *right_out)
{
    int i;

    if(!valid)
    {
        return;
    }

    for(i = 0; i < FLUID_BUFSIZE; ++i)
    {
        float left = 0.0f;
        float right = 0.0f;
        fluid_real_t out_left;
        fluid_real_t out_right;

        fluid_lexverb_process_sample(this, (float)in[i], &left, &right);

        out_left = (fluid_real_t)left * wet1 + (fluid_real_t)right * wet2;
        out_right = (fluid_real_t)right * wet1 + (fluid_real_t)left * wet2;

        left_out[i] += out_left;
        right_out[i] += out_right;
    }
}

void fluid_revmodel_lexverb::processreplace(const fluid_real_t *in, fluid_real_t *left_out,
                                            fluid_real_t *right_out)
{
    int i;

    if(!valid)
    {
        return;
    }

    for(i = 0; i < FLUID_BUFSIZE; ++i)
    {
        float left = 0.0f;
        float right = 0.0f;
        fluid_real_t out_left;
        fluid_real_t out_right;

        fluid_lexverb_process_sample(this, (float)in[i], &left, &right);

        out_left = (fluid_real_t)left * wet1 + (fluid_real_t)right * wet2;
        out_right = (fluid_real_t)right * wet1 + (fluid_real_t)left * wet2;

        left_out[i] = out_left;
        right_out[i] = out_right;
    }
}

void fluid_revmodel_lexverb::reset()
{
    if(!valid)
    {
        return;
    }

    fluid_lexverb_clear_blocks(this);
}

void fluid_revmodel_lexverb::set(int set, fluid_real_t roomsize, fluid_real_t damping,
                                 fluid_real_t width, fluid_real_t level)
{
    if(set & FLUID_REVMODEL_SET_ROOMSIZE)
    {
        fluid_clip(roomsize, 0.0f, 1.0f);
        this->roomsize = roomsize;
    }

    if(set & FLUID_REVMODEL_SET_DAMPING)
    {
        fluid_clip(damping, 0.0f, 1.0f);
        this->damp = damping;
    }

    if(set & FLUID_REVMODEL_SET_WIDTH)
    {
        fluid_clip(width, 0.0f, 100.0f);
        this->width = width;
    }

    if(set & FLUID_REVMODEL_SET_LEVEL)
    {
        fluid_clip(level, 0.0f, 1.0f);
        this->level = level;
    }

    fluid_lexverb_update(this);
}

int fluid_revmodel_lexverb::samplerate_change(fluid_real_t sample_rate)
{
    if(sample_rate <= 0.0f)
    {
        return FLUID_FAILED;
    }

    fluid_lexverb_release_blocks(this);

    if(fluid_lexverb_setup_blocks(this, sample_rate) != FLUID_OK)
    {
        fluid_lexverb_release_blocks(this);
        valid = false;
        FLUID_LOG(FLUID_ERR, "LEXverb reverb: failed to reinitialize delay lines");
        return FLUID_FAILED;
    }

    valid = true;
    return FLUID_OK;
}

fluid_revmodel_t *new_fluid_revmodel_lexverb(fluid_real_t sample_rate)
{
    return new(std::nothrow) fluid_revmodel_lexverb(sample_rate);
}
