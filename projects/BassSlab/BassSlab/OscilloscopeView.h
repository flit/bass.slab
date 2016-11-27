//
//  OscilloscopeView.h
//  BassSlab
//
//  Created by Chris Reed on 11/12/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import "SampleCollector.h"

@interface OscilloscopeView : NSView
{
    NSImage *_wave;
    slab::SampleCollector *_collector;
}

- (nonnull instancetype)initWithFrame:(NSRect)frameRect;
- (nullable instancetype)initWithCoder:(NSCoder * _Nonnull)coder;

- (void)updateWithBuffer:(float * _Nonnull)buffer count:(uint32_t)count;
- (void)drawBufferWithOffset:(uint32_t)offset count:(uint32_t)count;

@end
