//
//  OscilloscopeView.h
//  BassSlab
//
//  Created by Chris Reed on 11/12/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#import <Cocoa/Cocoa.h>

@interface OscilloscopeView : NSView
{
    NSImage *_wave;
}

- (void)updateWithBuffer:(float *)buffer count:(uint32_t)count;

@end
