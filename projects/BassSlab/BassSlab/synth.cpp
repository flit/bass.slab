//
//  synth.cpp
//  BassSlab
//
//  Created by Chris Reed on 11/10/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#include <stdio.h>
#include "synth.h"

using namespace slab;

Synth::Synth(float sampleRate)
:   _sampleRate(sampleRate)
{
    _mixBuf = std::unique_ptr<float[]>(new float[1024]);
    _renderBuf = std::unique_ptr<float[]>(new float[1024]);
}

Synth::~Synth()
{
}

void Synth::init()
{
    _kickSeq.set_sample_rate(_sampleRate);
    _kickSeq.set_tempo(100.0f);
    _kickSeq.set_sequence("x---x---x---x-x-x---x---x---x---xx--x--x--xxx-x-");
//    _kickSeq.set_sequence("x---------x---------"); //x-x----xx---");
    _kickSeq.init();

    _kickGen.set_sample_rate(_sampleRate);
    _kickGen.set_sequence(&_kickSeq);
    _kickGen.set_freq(120.0f);
    _kickGen.enable_sustain(false);
    _kickGen.init();
    _kickGen.set_attack(0.01f);
    _kickGen.set_release(0.6f);

    _bassSeq.set_sample_rate(_sampleRate);
    _bassSeq.set_tempo(100.0f);
    _bassSeq.set_sequence("--s20>>>>p--------"); //"--s>>>p-----s>>>>>>p----");
    _bassSeq.init();

    _bassGen.set_sample_rate(_sampleRate);
    _bassGen.set_sequence(&_bassSeq);
    _bassGen.set_freq(250.0f);
    _bassGen.enable_sustain(true);
    _bassGen.init();
    _bassGen.set_attack(0.3f);
    _bassGen.set_release(3.0f);

    _filter.set_sample_rate(_sampleRate);
    _filter.set_frequency(120.0f);
    _filter.set_q(0.4f);
    _filter.recompute_coefficients();
    _filter.set_input(&_bassGen);

    _delay.set_sample_rate(_sampleRate);
    _delay.set_maximum_delay_seconds(0.4f);
    _delay.set_delay_samples(_kickSeq.get_samples_per_beat());
    _delay.set_feedback(0.7f);
    _delay.set_wet_mix(0.5f);
    _delay.set_dry_mix(0.8f);
    _delay.set_input(&_kickGen);

    AudioBuffer mixBuf(_mixBuf.get(), 1024);
    _mixer.set_buffer(mixBuf);
    _mixer.set_input_count(2);
    _mixer.set_input(0, &_delay, 0.5f);
    _mixer.set_input(1, &_bassGen, 0.34f);
}

void Synth::reinit(const char * sequence)
{
    SequenceInfo * seq = _reader.parse(sequence);
    if (!seq)
    {
        return;
    }

    _kickSeq.set_sequence(seq->channels[0]);
    _kickSeq.set_tempo(seq->tempo);
    _bassSeq.set_sequence(seq->channels[1]);
    _bassSeq.set_tempo(seq->tempo);

    _kickSeq.init();
    _kickGen.init();
    _bassSeq.init();
    _bassGen.init();

    _delay.set_delay_samples(_kickSeq.get_samples_per_beat());

    delete seq;
}

void Synth::render(float *samples, uint32_t count)
{
    _mixer.process(samples, count);
}

