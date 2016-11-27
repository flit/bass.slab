//
//  OscilloscopeView.m
//  BassSlab
//
//  Created by Chris Reed on 11/12/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#import "OscilloscopeView.h"
#import <QuartzCore/QuartzCore.h>
#import <math.h>

static const uint32_t kSampleCount = 8192;

@implementation OscilloscopeView

- (nonnull instancetype)initWithFrame:(NSRect)frameRect
{
    self = [super initWithFrame:frameRect];
    if (self)
    {
        self.layer.backgroundColor = CGColorGetConstantColor(kCGColorBlack);

        _collector = new slab::SampleCollector;
        _collector->init(kSampleCount);
    }
    return self;
}

- (nullable instancetype)initWithCoder:(NSCoder *)coder
{
    self = [super initWithCoder:coder];
    if (self)
    {
        self.layer.backgroundColor = CGColorGetConstantColor(kCGColorBlack);

        _collector = new slab::SampleCollector;
        _collector->init(kSampleCount);
    }
    return self;
}

- (void)updateWithBuffer:(float *)buffer count:(uint32_t)count
{
    _collector->append(slab::AudioBuffer(buffer, count));

    uint32_t max = _collector->get_count();
    if (max >= 2048)
    {
        uint32_t i;
        float peak = 0;
        for (i = 0; i < max; ++i)
        {
            float s = fabs((*_collector)[i]);
            if (s > peak)
            {
                peak = s;
            }
        }

        if (peak >= 0.15)
        {
            for (i = 1; i < max; ++i)
            {
                float s = (*_collector)[i];
                float prev = (*_collector)[i - 1];
                float delta = fabs(s) - fabs(prev);

                if (delta < 0 && s > 0.15)
                {
                    [self drawBufferWithOffset:i - 1 count:max - i];
                    _collector->clear();
                    return;
                }
            }
        }
        else
        {
            [self drawBufferWithOffset:0 count:max];
            _collector->clear();
        }
    }
}

- (void)drawBufferWithOffset:(uint32_t)offset count:(uint32_t)count
{
    NSImage *img = [[NSImage alloc] initWithSize:self.frame.size];

    [img lockFocus];

    [[NSColor blackColor] set];
    [NSBezierPath fillRect:self.bounds];

    float width = NSWidth(self.frame);
    float height = NSHeight(self.frame);
    float ymid = height / 2.0;
    uint32_t sampleDelta = 1;
    while ((count / sampleDelta) > width)
    {
        ++sampleDelta;
    }
    float pointDelta = width / (float)(count / sampleDelta);

    [[NSColor grayColor] set];
    [NSBezierPath strokeLineFromPoint:NSMakePoint(0, ymid) toPoint:NSMakePoint(width, ymid)];

    [[NSColor greenColor] set];

    NSBezierPath *path = [NSBezierPath bezierPath];
    [path moveToPoint:NSMakePoint(0.0, ymid)];

    uint32_t i = offset;
    float x = 0.0;
    while (count && x < width)
    {
        float sample = (*_collector)[i];
        if (sample > 1.0)
        {
            sample = 1.0;
        }
        else if (sample < -1.0)
        {
            sample = -1.0;
        }
        float y = ymid + (sample * ymid);

        if (i == offset)
        {
            [path moveToPoint:NSMakePoint(x, y)];
        }
        else
        {
            [path lineToPoint:NSMakePoint(x, y)];
        }

        i += sampleDelta;
        count -= sampleDelta;
        x += pointDelta;
    }

    path.lineWidth = 3.5;
    [path stroke];
    [img unlockFocus];
    _wave = img;

    [CATransaction begin];
    self.layer.contents = img;
    [CATransaction commit];
}

- (BOOL)wantsLayer
{
    return YES;
}

- (BOOL)wantsUpdateLayer
{
    return YES;
}

- (void)updateLayer
{
    if (_wave)
    {
        self.layer.contents = _wave;
    }
    else
    {
        self.layer.backgroundColor = CGColorGetConstantColor(kCGColorBlack);
    }
}

- (void)drawRect:(NSRect)dirtyRect
{
    [super drawRect:dirtyRect];

    if (_wave)
    {
        [_wave drawInRect:self.bounds];
    }
    else
    {
        [[NSColor blackColor] set];
        [NSBezierPath fillRect:self.bounds];
    }
}

@end
