/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Various Functions for color space conversions.
 *
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */


#ifndef __COLOR_SPACE_CONVERSION_H_INCLUDED__
#define __COLOR_SPACE_CONVERSION_H_INCLUDED__

#include <cstring> // memcpy




/* Read this webside, helps to understand some of the image coversion stuff.
http://www.fourcc.org/fccyvrgb.php
For infos to the following code see opencv cap_v4l-cpp
*/





/* calmp: convert a 16.16 fixed-point value to a byte, with clipping. */
template< typename T >
inline uint8_t clamp( const T value )
{
	return ( ( value )>0xffffff? 0xff : ( ( value )<=0xffff ?0:(( value )>>16)));
};





static inline void move_420_block( int yTL, int yTR, int yBL, int yBR, const int u, const int v, const int rowPixels, uint8_t* rgb )
{
    const int rvScale = 91881;
    const int guScale = -22553;
    const int gvScale = -46801;
    const int buScale = 116129;
    const int yScale  = 65536;
    int r, g, b;

    g = guScale * u + gvScale * v;
//  if (force_rgb) {
//      r = buScale * u;
//      b = rvScale * v;
//  } else {
        r = rvScale * v;
        b = buScale * u;
//  }

    yTL *= yScale; yTR *= yScale;
    yBL *= yScale; yBR *= yScale;

    /* Write out top two pixels */
    rgb[0] = clamp(b+yTL); rgb[1] = clamp(g+yTL);
    rgb[2] = clamp(r+yTL);

    rgb[3] = clamp(b+yTR); rgb[4] = clamp(g+yTR);
    rgb[5] = clamp(r+yTR);

    /* Skip down to next line to write out bottom two pixels */
    rgb += 3 * rowPixels;
    rgb[0] = clamp(b+yBL); rgb[1] = clamp(g+yBL);
    rgb[2] = clamp(r+yBL);

    rgb[3] = clamp(b+yBR); rgb[4] = clamp(g+yBR);
    rgb[5] = clamp(r+yBR);
}


template< uint32_t FROM, uint32_t TO > //= V4L2_PIX_FMT_BGR24 >
static void convert( const std::size_t width, const std::size_t height, const uint8_t* pIn0, uint8_t* pOut0 )
{
/* empty prototype, should be implemented for the different color conversion functions*/
};


template< >
void convert< V4L2_PIX_FMT_BGR24, V4L2_PIX_FMT_BGR24 >( const std::size_t width, const std::size_t height, const uint8_t* pSource, uint8_t* pDest )
{
	const std::size_t size = width * height * 3;
	std::memcpy ( pDest, pSource, size );
}
	
template< >
void convert< V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_BGR24 >( const std::size_t width, const std::size_t height, const uint8_t* pIn0, uint8_t* pOut0 )
{
	const std::size_t numpix = width * height;
	const int bytes = 24 >> 3;
	int y00, y01, y10, y11;
	int u, v;
	const uint8_t *pY = pIn0;
	const uint8_t *pU = pY + numpix;
	const uint8_t *pV = pU + numpix / 4;
	uint8_t *pOut = pOut0;

	for ( std::size_t j = 0; j <= height - 2; j += 2 )
	{
		for( std::size_t i = 0; i <= width - 2; i += 2 )
		{
			y00 = *pY;
			y01 = *(pY + 1);
			y10 = *(pY + width);
			y11 = *(pY + width + 1);
			u = (*pU++) - 128;
			v = (*pV++) - 128;

			move_420_block(y00, y01, y10, y11, u, v, width, pOut);

			pY += 2;
			pOut += 2 * bytes;

		}
		pY += width;
		pOut += width * bytes;
	}
}

inline int clip(int value) {
    return (value > 255) ? 255 : (value < 0) ? 0 : value;
}

/** the following code is take from could be an alternative to above one:
// http://stackoverflow.com/questions/8836872/mjpeg-to-raw-rgb24-with-video4linux
*/

static void yuv420_to_rgb24(
/* luminance (source) */const uint8_t* const y
/* u chrominance (source) */, const uint8_t* u
/* v chrominance (source) */, const uint8_t* v
/* rgb interleaved (destination) */, uint8_t* const dst
/* jpeg size */, int const size
/* image width */, int const width) {

    const int lineSize = width * 3;

    uint8_t* r1 = dst;
    uint8_t* g1 = r1 + 1;
    uint8_t* b1 = r1 + 2;

    uint8_t* r2 = r1 + lineSize;
    uint8_t* g2 = r2 + 1;
    uint8_t* b2 = r2 + 2;

    const uint8_t* y1 = y;
    const uint8_t* y2 = y + width;

    uint8_t* const end = r1 + size;

    int c1 = 0;
    int c2 = 0;
    int e = 0;
    int d = 0;

    while (r1 != end) {
        uint8_t* const lineEnd = r2;
        /* line by line */
        while (r1 != lineEnd) {
            /* first pixel */
            c1 = *y1 - 16;
            c2 = *y2 - 16;
            d = *u - 128;
            e = *v - 128;

            *r1 = clip(c1 + ((454 * e) >> 8));
            *g1 = clip(c1 - ((88 * e + 183 * d) >> 8));
            *b1 = clip(c1 + ((359 * d) >> 8));

            *r2 = clip(c2 + ((454 * e) >> 8));
            *g2 = clip(c2 - ((88 * e + 183 * d) >> 8));
            *b2 = clip(c2 + ((359 * d) >> 8));

            r1 += 3;
            g1 += 3;
            b1 += 3;

            r2 += 3;
            g2 += 3;
            b2 += 3;

            ++y1;
            ++y2;

            /* second pixel */

            c1 = *y1 - 16;
            c2 = *y2 - 16;
            d = *u - 128;
            e = *v - 128;

            *r1 = clip(c1 + ((454 * e) >> 8));
            *g1 = clip(c1 - ((88 * e + 183 * d) >> 8));
            *b1 = clip(c1 + ((359 * d) >> 8));

            *r2 = clip(c2 + ((454 * e) >> 8));
            *g2 = clip(c2 - ((88 * e + 183 * d) >> 8));
            *b2 = clip(c2 + ((359 * d) >> 8));

            r1 += 3;
            g1 += 3;
            b1 += 3;

            r2 += 3;
            g2 += 3;
            b2 += 3;

            ++y1;
            ++y2;

            ++u;
            ++v;
        }
        r1 += lineSize;
        g1 += lineSize;
        b1 += lineSize;
        r2 += lineSize;
        g2 += lineSize;
        b2 += lineSize;
        y1 += width;
        y2 += width;
    }
}

#endif // __COLOR_SPACE_CONVERSION_H_INCLUDED__
