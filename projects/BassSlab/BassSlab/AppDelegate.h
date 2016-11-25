//
//  AppDelegate.h
//  BassSlab
//
//  Created by Chris Reed on 11/5/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import <AudioToolbox/AudioQueue.h>
#include "synth.h"

@interface AppDelegate : NSObject <NSApplicationDelegate>
{
    AudioQueueRef _audioQueue;
    bool _isPlaying;
    slab::Synth *_synth;
    AudioQueueBufferRef _buffers[3];
}

- (void)queueCallback:(AudioQueueRef _Nonnull)inAQ buffer:(AudioQueueBufferRef _Nonnull)inBuffer;

- (void)siphon:(AudioQueueProcessingTapRef)inAQTap inFrames:(UInt32)inNumberFrames timeStamp:(AudioTimeStamp *)ioTimeStamp outFrames:(UInt32 *)outNumberFrames data:(AudioBufferList *)ioData;

@end

