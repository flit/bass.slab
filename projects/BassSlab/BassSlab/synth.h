//
//  synth.h
//  BassSlab
//
//  Created by Chris Reed on 11/10/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#ifndef synth_h
#define synth_h

#include "audio_filter.h"
#include "audio_ramp.h"
#include "asr_envelope.h"
#include "sine_osc.h"
#include "sequencer.h"
#include "audio_mixer.h"
#include "delay_line.h"
#include "rbj_filter.h"
#include <memory>

namespace slab {

class Synth
{
public:
    Synth(float sampleRate);
    ~Synth();

    void init();
    
    void render(float *samples, uint32_t count);

private:
    float _sampleRate;
    std::unique_ptr<float[]> _mixBuf;
    std::unique_ptr<float[]> _renderBuf;

    SineGenerator _kickGen;
    Sequencer _kickSeq;
    SineGenerator _bassGen;
    Sequencer _bassSeq;
    AudioMixer _mixer;
    RBJFilter _filter;
    DelayLine _delay;

};

} // namespace slab

#endif /* synth_h */
