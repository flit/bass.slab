//
//  OscilloscopeView.m
//  BassSlab
//
//  Created by Chris Reed on 11/12/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#import "OscilloscopeView.h"
#import <QuartzCore/QuartzCore.h>

@implementation OscilloscopeView

- (instancetype)initWithFrame:(NSRect)frameRect
{
    self = [super initWithFrame:frameRect];
    if (self)
    {
        self.layer.backgroundColor = CGColorGetConstantColor(kCGColorBlack);
    }
    return self;
}

- (void)updateWithBuffer:(float *)buffer count:(uint32_t)count
{
    NSImage *img = [[NSImage alloc] initWithSize:self.frame.size];

    [img lockFocus];

    [[NSColor blackColor] set];
    [NSBezierPath fillRect:self.bounds];
    [[NSColor whiteColor] set];

    float width = NSWidth(self.frame);
    float height = NSHeight(self.frame);
    float ymid = height / 2.0;
    uint32_t sampleDelta = 1;
    float pointDelta = width / (float)count;
//    if (count >= width)
//    {
//        // More samples than pixels wide.
//        sampleDelta =
//    }
//    else
//    {
//    }

    NSBezierPath *path = [NSBezierPath bezierPath];
    [path moveToPoint:NSMakePoint(0.0, ymid)];

    float miny = 1.0, maxy = -1.0;
    float x = 0.0;
    while (count && x < width)
    {
        float sample = *buffer;
 
        if (sample < miny)
        {
            miny = sample;
        }
        if (sample > maxy)
        {
            maxy = sample;
        }

        if (sample > 1.0)
        {
            sample = 1.0;
        }
        else if (sample < -1.0)
        {
            sample = -1.0;
        }
        float y = ymid - (sample * ymid);

        [path lineToPoint:NSMakePoint(x, y)];

        count -= sampleDelta;
        buffer += sampleDelta;
        x += pointDelta;
    }

    [path stroke];
    [img unlockFocus];
    _wave = img;

    [CATransaction begin];
    self.layer.contents = img;
    [CATransaction commit];

//    NSLog(@"miny = %g; maxy = %g", miny, maxy);
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
