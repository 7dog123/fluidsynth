#pragma once

#include <array>

#include "fluid_rev.h"
#include "fluid_rev_filters.h"

struct fluid_revmodel_dattorro : public _fluid_revmodel_t
{
    fluid_real_t roomsize;
    fluid_real_t damp;
    fluid_real_t level;
    fluid_real_t wet1;
    fluid_real_t wet2;
    fluid_real_t width;
    fluid_real_t bandwidth;
    fluid_real_t decay;
    fluid_real_t cached_sample_rate;
    float bandwidth_state;
    float damp_state_left;
    float damp_state_right;

    fluid_reverb_delay_line<float> predelay;
    fluid_reverb_allpass<float> input_ap[4];
    fluid_reverb_allpass<float> tank_ap[4];
    fluid_reverb_delay_line<float> tank_delay[4];
    std::array<int, 14> taps;

    explicit fluid_revmodel_dattorro(fluid_real_t sample_rate);
    ~fluid_revmodel_dattorro() override;

    void processmix(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out) override;
    void processreplace(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out) override;
    void reset() override;
    void set(int set, fluid_real_t roomsize, fluid_real_t damping,
             fluid_real_t width, fluid_real_t level) override;
    int samplerate_change(fluid_real_t sample_rate) override;

private:
    void setup();
    void update();

    template<bool MIX>
    void process(const fluid_real_t *in, fluid_real_t *left_out, fluid_real_t *right_out);
};

typedef struct fluid_revmodel_dattorro fluid_revmodel_dattorro_t;
