// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _IMAGEWRITE_H_
#define _IMAGEWRITE_H_

#include <string>

/*
	Minimal PNG writer for the snapshots (-snapshot): 8-bit RGB, zlib-compressed,
	no external image library. 'rgb' holds width * height * 3 bytes with the rows
	bottom-up, as glReadPixels returns them; the file is written top-down.
	Returns false if the file cannot be written.
*/
bool writePNG(const std::string& path, int width, int height, const unsigned char* rgb);

// Read the current back buffer (width x height) and write it with writePNG.
bool saveFramebufferPNG(const std::string& path, int width, int height);

#endif // _IMAGEWRITE_H_
