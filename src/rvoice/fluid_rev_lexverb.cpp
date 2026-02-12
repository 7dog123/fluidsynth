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

#include "fluid_sys.h"
#include "fluid_rev_lexverb.h"

constexpr float LEX_TRIM = 0.7f;
constexpr float LEX_SCALE_WET_WIDTH = 0.2f;

static int fluid_lexverb_ms_to_buf_length(float ms, fluid_real_t sample_rate)
{
    return ms * (sample_rate * (1 / 1000.0f));
}

static void fluid_lexverb_setup_blocks(fluid_revmodel_lexverb_t *rev, fluid_real_t sample_rate)
{
    int i;
    for(i = 0; i < NUM_OF_AP_SECTS; ++i)
    {
        int length = fluid_lexverb_ms_to_buf_length(LEX_REVERB_PARMS[i].length, sample_rate);

        rev->ap[i].set_mode(FLUID_REVERB_ALLPASS_SCHROEDER);
        rev->ap[i].set_feedback(LEX_REVERB_PARMS[i].coef);
        rev->ap[i].set_buffer(length);
        rev->ap[i].set_index(1);
        rev->ap[i].set_last_output(0.0f);
    }

    for(i = 0; i < NUM_OF_DELAY_SECTS; ++i)
    {
        int index = NUM_OF_AP_SECTS + i;
        int length = fluid_lexverb_ms_to_buf_length(LEX_REVERB_PARMS[index].length, sample_rate);

        rev->dl[i].set_coefficient(LEX_REVERB_PARMS[index].coef);
        rev->dl[i].set_buffer(length);
        rev->dl[i].set_positions(1, 1);
        rev->dl[i].set_last_output(0.0f);
    }

    this->reset();
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
    fluid_reverb_allpass<float> *ap = rev->ap;
    fluid_reverb_delay_line<float> *dl = rev->dl;
    float output;

    output = ap[0].process(input * LEX_TRIM); // technically, the left input sample should be here
    output = ap[1].process(output);
    output = ap[2].process(output + dl[1].process(ap[9].get_last_output()) * dl[1].get_coefficient());
    output = ap[3].process(output);
    output = ap[4].process(output);
    *out_left = output;

    output = ap[5].process(input * LEX_TRIM); // technically, the right input sample should be here
    output = ap[6].process(output);
    output = ap[7].process(output + dl[0].process(ap[4].get_last_output()) * dl[0].get_coefficient());
    output = ap[8].process(output);
    output = ap[9].process(output);
    *out_right = output;

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
      damp_state_left(0.0f),
      damp_state_right(0.0f)
{
    if(sample_rate <= 0.0f)
    {
        throw std::invalid_argument("Sample rate must be positive");
    }

    fluid_lexverb_setup_blocks(this, sample_rate);
}

fluid_revmodel_lexverb::~fluid_revmodel_lexverb() = default;


void fluid_revmodel_lexverb::processmix(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out)
{
    process<true>(in, left_out, right_out);
}

void fluid_revmodel_lexverb::processreplace(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out)
{
    process<false>(in, left_out, right_out);
}

template<bool MIX>
void fluid_revmodel_lexverb::process(const fluid_real_t *in, fluid_real_t *left_out,
                                     fluid_real_t *right_out)
{
    int i;

    for(i = 0; i < FLUID_BUFSIZE; ++i)
    {
        float left = 0.0f;
        float right = 0.0f;

        fluid_lexverb_process_sample(this, (float)in[i], &left, &right);

        fluid_real_t out_left = left * wet1 + right * wet2;
        fluid_real_t out_right = right * wet1 + left * wet2;

        if(MIX)
        {
            left_out[i] += out_left;
            right_out[i] += out_right;
        }
        else
        {
            left_out[i] = out_left;
            right_out[i] = out_right;
        }
    }
}

void fluid_revmodel_lexverb::reset()
{
    int i;

    for(i = 0; i < NUM_OF_AP_SECTS; ++i)
    {
        if(rev->ap[i].has_buffer())
        {
            rev->ap[i].fill_buffer(0.0f);
            rev->ap[i].set_index(1);
        }
        rev->ap[i].set_last_output(0.0f);
    }

    for(i = 0; i < NUM_OF_DELAY_SECTS; ++i)
    {
        if(rev->dl[i].has_buffer())
        {
            rev->dl[i].fill_buffer(0.0f);
            rev->dl[i].set_positions(1, 1);
        }
        rev->dl[i].set_last_output(0.0f);
    }

    rev->damp_state_left = 0.0f;
    rev->damp_state_right = 0.0f;
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
    FLUID_LOG(FLUID_ERR, "LEXverb reverb: sample rate change is not supported");
    return FLUID_FAILED;
}
