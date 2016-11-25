//
//  AppDelegate.m
//  BassSlab
//
//  Created by Chris Reed on 11/5/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#import "AppDelegate.h"
#import "OscilloscopeView.h"

const float kSampleRate = 48000.0f;
const uint32_t kChannelCount = 1;
const uint32_t kBufferSize = 1024;

@interface AppDelegate ()

@property (weak) IBOutlet NSWindow *window;
@property (weak) IBOutlet NSButton *startStopButton;
@property (weak) IBOutlet OscilloscopeView *oscope;

@end

void MyAudioQueuePropertyListenerProc(void * inUserData, AudioQueueRef inAQ, AudioQueuePropertyID inID)
{
	UInt32 isRunning = 0;
	UInt32 size = sizeof(isRunning);
	AudioQueueGetProperty(inAQ, kAudioQueueProperty_IsRunning, &isRunning, &size);
    NSLog(@"isRunning = %d", isRunning);
}

void MyAudioQueueProcessingTapCallback(
        void *                          inClientData,
        AudioQueueProcessingTapRef      inAQTap,
        UInt32                          inNumberFrames,
        AudioTimeStamp *                ioTimeStamp,
        AudioQueueProcessingTapFlags *  ioFlags,
        UInt32 *                        outNumberFrames,
        AudioBufferList *               ioData)
{
    AppDelegate *_self = (__bridge AppDelegate *)inClientData;
    [_self siphon:inAQTap inFrames:inNumberFrames timeStamp:ioTimeStamp outFrames:outNumberFrames data:ioData];
}

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    _synth = new slab::Synth(kSampleRate);
    _synth->init();

    AudioStreamBasicDescription format;
    format.mSampleRate = kSampleRate;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagsNativeFloatPacked;
    format.mBytesPerPacket = sizeof(float) * kChannelCount;
    format.mFramesPerPacket = 1;
    format.mBytesPerFrame = sizeof(float) * kChannelCount;
    format.mChannelsPerFrame = kChannelCount;
    format.mBitsPerChannel = 32;

    OSStatus status = AudioQueueNewOutputWithDispatchQueue(&_audioQueue, &format, 0, dispatch_queue_create("audio", NULL),
        ^(AudioQueueRef  _Nonnull inAQ, AudioQueueBufferRef  _Nonnull inBuffer) {
            [self queueCallback:inAQ buffer:inBuffer];
        });
    if (status)
    {
        NSLog(@"AudioQueueNewOutputWithDispatchQueue = %d", status);
    }

    AudioQueueAddPropertyListener(_audioQueue, kAudioQueueProperty_IsRunning, MyAudioQueuePropertyListenerProc, NULL);

    AudioQueueSetParameter(_audioQueue, kAudioQueueParam_Volume, 1.0);

//    UInt32 maxFrames;
//    AudioStreamBasicDescription tapFormat;
//    AudioQueueProcessingTapRef tap;
//    status = AudioQueueProcessingTapNew(_audioQueue, MyAudioQueueProcessingTapCallback, (__bridge void *)self, kAudioQueueProcessingTap_Siphon, &maxFrames, &tapFormat, &tap);
//    if (status)
//    {
//        NSLog(@"AudioQueueProcessingTapNew = %d", status);
//    }

    // Allocate buffers and prime the queue.
    AudioQueueBufferRef buf;
    int i;
//    _isPlaying = true;
    for (i = 0; i < 3; ++i)
    {
        AudioQueueAllocateBuffer(_audioQueue, kBufferSize, &buf);
        _buffers[i] = buf;

//        [self queueCallback:_audioQueue buffer:buf];
    }
//    _isPlaying = false;
}

- (void)applicationWillTerminate:(NSNotification *)aNotification
{
    AudioQueueDispose(_audioQueue, true);
    _isPlaying = false;
}

- (IBAction)startStopAudio:(id)sender
{
    OSStatus status;
    if (!_isPlaying)
    {
        _isPlaying = true;

        // Prime the queue.
        int i;
        for (i = 0; i < 3; ++i)
        {
            [self queueCallback:_audioQueue buffer:_buffers[i]];
        }

        status = AudioQueueStart(_audioQueue, NULL);
        if (status)
        {
            NSLog(@"AQStart = %d", status);
        }
        _startStopButton.title = @"Stop";
    }
    else
    {
        _isPlaying = false;
        status = AudioQueueStop(_audioQueue, false);
        if (status)
        {
            NSLog(@"AQStop = %d", status);
        }
        _startStopButton.title = @"Start";
    }
}

- (void)queueCallback:(AudioQueueRef _Nonnull)inAQ buffer:(AudioQueueBufferRef _Nonnull)inBuffer
{
    uint32_t sampleCount = inBuffer->mAudioDataBytesCapacity / sizeof(float) / kChannelCount;
    float *buffer = (float *)inBuffer->mAudioData;
    _synth->render(buffer, sampleCount);
    inBuffer->mAudioDataByteSize = inBuffer->mAudioDataBytesCapacity;

    [_oscope updateWithBuffer:buffer count:sampleCount];

    if (_isPlaying)
    {
        OSStatus status = AudioQueueEnqueueBuffer(_audioQueue, inBuffer, 0, NULL);
        if (status)
        {
            NSLog(@"AudioQueueEnqueueBuffer = %d", status);
        }
    }
}

- (void)siphon:(AudioQueueProcessingTapRef)inAQTap inFrames:(UInt32)inNumberFrames timeStamp:(AudioTimeStamp *)ioTimeStamp outFrames:(UInt32 *)outNumberFrames data:(AudioBufferList *)ioData
{
    NSLog(@"siphon tap: %g", ioTimeStamp->mSampleTime);
}

@end
