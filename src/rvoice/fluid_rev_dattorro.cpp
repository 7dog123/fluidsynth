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

#include "fluid_rev_dattorro.h"
#include <stdexcept>

namespace
{
constexpr float DATTORRO_TRIM = 0.6f;
constexpr float DATTORRO_SCALE_WET_WIDTH = 0.2f;
constexpr float DATTORRO_INPUT_DIFFUSION1 = 0.75f;
constexpr float DATTORRO_INPUT_DIFFUSION2 = 0.625f;
constexpr float DATTORRO_DECAY_DIFFUSION1 = 0.7f;
constexpr float DATTORRO_DECAY_DIFFUSION2 = 0.5f;
constexpr float DATTORRO_PREDELAY_MS = 4.0f;
constexpr float DATTORRO_DELAY_S[] =
{
    0.004771345f, 0.003595309f, 0.012734787f, 0.009307483f,
    0.022579886f, 0.149625349f, 0.060481839f, 0.1249958f,
    0.030509727f, 0.141695508f, 0.089244313f, 0.106280031f
};
constexpr float DATTORRO_TAP_S[] =
{
    0.008937872f, 0.099929438f, 0.064278754f, 0.067067639f, 0.066866033f, 0.006283391f, 0.035818689f,
    0.011861161f, 0.121870905f, 0.041262054f, 0.08981553f, 0.070931756f, 0.011256342f, 0.004065724f
};

static int fluid_dattorro_seconds_to_samples(float seconds, fluid_real_t sample_rate)
{
    int length = static_cast<int>(seconds * sample_rate + 0.5f);
    return length > 0 ? length : 1;
}

static float fluid_dattorro_read_tap(const fluid_reverb_delay_line<float> &delay, int tap)
{
    int size = delay.size();
    if(size <= 0)
    {
        return 0.0f;
    }

    int index = delay.line_out + tap;
    index %= size;
    if(index < 0)
    {
        index += size;
    }
    return delay.line[index];
}

static float fluid_dattorro_read_tap(const fluid_reverb_allpass<float> &ap, int tap)
{
    return fluid_dattorro_read_tap(ap.delay, tap);
}
}

fluid_revmodel_dattorro::fluid_revmodel_dattorro(fluid_real_t sample_rate)
    : roomsize(0.0f),
      damp(0.0f),
      level(0.0f),
      wet1(0.0f),
      wet2(0.0f),
      width(0.0f),
      bandwidth(0.9999f),
      decay(0.5f),
      cached_sample_rate(sample_rate),
      bandwidth_state(0.0f),
      damp_state_left(0.0f),
      damp_state_right(0.0f)
{
    if(sample_rate <= 0.0f)
    {
        throw std::invalid_argument("Sample rate must be positive");
    }
    setup();
}

fluid_revmodel_dattorro::~fluid_revmodel_dattorro() = default;

void fluid_revmodel_dattorro::setup()
{
    predelay.set_buffer(fluid_dattorro_seconds_to_samples(DATTORRO_PREDELAY_MS / 1000.0f, cached_sample_rate));

    for(int i = 0; i < 4; ++i)
    {
        input_ap[i].set_mode(FLUID_REVERB_ALLPASS_SCHROEDER);
        input_ap[i].set_buffer(fluid_dattorro_seconds_to_samples(DATTORRO_DELAY_S[i], cached_sample_rate));
    }
    input_ap[0].set_feedback(DATTORRO_INPUT_DIFFUSION1);
    input_ap[1].set_feedback(DATTORRO_INPUT_DIFFUSION1);
    input_ap[2].set_feedback(DATTORRO_INPUT_DIFFUSION2);
    input_ap[3].set_feedback(DATTORRO_INPUT_DIFFUSION2);

    for(int i = 0; i < 4; ++i)
    {
        tank_ap[i].set_mode(FLUID_REVERB_ALLPASS_SCHROEDER);
        tank_ap[i].set_buffer(fluid_dattorro_seconds_to_samples(DATTORRO_DELAY_S[4 + i * 2], cached_sample_rate));
        tank_delay[i].set_buffer(fluid_dattorro_seconds_to_samples(DATTORRO_DELAY_S[5 + i * 2], cached_sample_rate));
    }
    tank_ap[0].set_feedback(DATTORRO_DECAY_DIFFUSION1);
    tank_ap[1].set_feedback(DATTORRO_DECAY_DIFFUSION2);
    tank_ap[2].set_feedback(DATTORRO_DECAY_DIFFUSION1);
    tank_ap[3].set_feedback(DATTORRO_DECAY_DIFFUSION2);

    for(size_t i = 0; i < taps.size(); ++i)
    {
        taps[i] = fluid_dattorro_seconds_to_samples(DATTORRO_TAP_S[i], cached_sample_rate);
    }

    reset();
}

void fluid_revmodel_dattorro::update()
{
    fluid_real_t wet = level / (1.0f + width * DATTORRO_SCALE_WET_WIDTH);
    wet1 = wet * (width / 2.0f + 0.5f);
    wet2 = wet * ((1.0f - width) / 2.0f);
    decay = 0.2f + roomsize * 0.78f;
}

void fluid_revmodel_dattorro::processmix(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out)
{
    process<true>(in, left_out, right_out);
}

void fluid_revmodel_dattorro::processreplace(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out)
{
    process<false>(in, left_out, right_out);
}

template<bool MIX>
void fluid_revmodel_dattorro::process(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out)
{
    float bandwidth_state_local = bandwidth_state;
    float damp_state_left_local = damp_state_left;
    float damp_state_right_local = damp_state_right;

    for(int i = 0; i < FLUID_BUFSIZE; ++i)
    {
        float input = static_cast<float>(in[i]) * DATTORRO_TRIM;
        float pre = predelay.process(input);
        bandwidth_state_local += static_cast<float>(bandwidth) * (pre - bandwidth_state_local);

        float split = input_ap[0].process(bandwidth_state_local);
        split = input_ap[1].process(split);
        split = input_ap[2].process(split);
        split = input_ap[3].process(split);

        float left = split + static_cast<float>(decay) * tank_delay[3].get_last_output();
        left = tank_ap[0].process(left);
        left = tank_delay[0].process(left);
        damp_state_left_local += (1.0f - static_cast<float>(damp)) * (left - damp_state_left_local);
        left = tank_ap[1].process(static_cast<float>(decay) * damp_state_left_local);
        left = tank_delay[1].process(left);

        float right = split + static_cast<float>(decay) * tank_delay[1].get_last_output();
        right = tank_ap[2].process(right);
        right = tank_delay[2].process(right);
        damp_state_right_local += (1.0f - static_cast<float>(damp)) * (right - damp_state_right_local);
        right = tank_ap[3].process(static_cast<float>(decay) * damp_state_right_local);
        right = tank_delay[3].process(right);

        float out_left = fluid_dattorro_read_tap(tank_delay[2], taps[0])
                         + fluid_dattorro_read_tap(tank_delay[2], taps[1])
                         - fluid_dattorro_read_tap(tank_ap[3], taps[2])
                         + fluid_dattorro_read_tap(tank_delay[3], taps[3])
                         - fluid_dattorro_read_tap(tank_delay[0], taps[4])
                         - fluid_dattorro_read_tap(tank_ap[1], taps[5])
                         - fluid_dattorro_read_tap(tank_delay[1], taps[6]);

        float out_right = fluid_dattorro_read_tap(tank_delay[0], taps[7])
                          + fluid_dattorro_read_tap(tank_delay[0], taps[8])
                          - fluid_dattorro_read_tap(tank_ap[1], taps[9])
                          + fluid_dattorro_read_tap(tank_delay[1], taps[10])
                          - fluid_dattorro_read_tap(tank_delay[2], taps[11])
                          - fluid_dattorro_read_tap(tank_ap[3], taps[12])
                          - fluid_dattorro_read_tap(tank_delay[3], taps[13]);

        fluid_real_t mix_left = out_left * wet1 + out_right * wet2;
        fluid_real_t mix_right = out_right * wet1 + out_left * wet2;

        if(MIX)
        {
            left_out[i] += mix_left;
            right_out[i] += mix_right;
        }
        else
        {
            left_out[i] = mix_left;
            right_out[i] = mix_right;
        }
    }

    bandwidth_state = bandwidth_state_local;
    damp_state_left = damp_state_left_local;
    damp_state_right = damp_state_right_local;
}

void fluid_revmodel_dattorro::reset()
{
    if(predelay.has_buffer())
    {
        predelay.fill_buffer(0.0f);
        predelay.set_single_tap_position(0);
    }

    for(int i = 0; i < 4; ++i)
    {
        if(input_ap[i].has_buffer())
        {
            input_ap[i].fill_buffer(0.0f);
            input_ap[i].set_index(0);
        }
        input_ap[i].set_last_output(0.0f);

        if(tank_ap[i].has_buffer())
        {
            tank_ap[i].fill_buffer(0.0f);
            tank_ap[i].set_index(0);
        }
        tank_ap[i].set_last_output(0.0f);

        if(tank_delay[i].has_buffer())
        {
            tank_delay[i].fill_buffer(0.0f);
            tank_delay[i].set_single_tap_position(0);
        }
        tank_delay[i].set_last_output(0.0f);
    }

    bandwidth_state = 0.0f;
    damp_state_left = 0.0f;
    damp_state_right = 0.0f;
}

void fluid_revmodel_dattorro::set(int set, fluid_real_t roomsize, fluid_real_t damping,
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

    update();
}

int fluid_revmodel_dattorro::samplerate_change(fluid_real_t sample_rate)
{
    if(sample_rate <= 0.0f)
    {
        return FLUID_FAILED;
    }

    cached_sample_rate = sample_rate;
    setup();
    update();
    return FLUID_OK;
}
