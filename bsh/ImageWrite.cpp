// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "ImageWrite.h"
#include <algorithm>
#include <cstdio>
#include <vector>
#include <zlib.h>

// append a big-endian 32-bit value
static void put32(std::vector<unsigned char>& b, unsigned int v){
	b.push_back((unsigned char)(v >> 24)); b.push_back((unsigned char)(v >> 16));
	b.push_back((unsigned char)(v >> 8));  b.push_back((unsigned char)v);
}

// append a PNG chunk: length, type, data, CRC over type + data
static void putChunk(std::vector<unsigned char>& out, const char* type, const std::vector<unsigned char>& data){
	put32(out, (unsigned int)data.size());
	std::vector<unsigned char> td(type, type + 4);
	td.insert(td.end(), data.begin(), data.end());
	out.insert(out.end(), td.begin(), td.end());
	put32(out, (unsigned int)crc32(0L, td.data(), (uInt)td.size()));
}

bool writePNG(const std::string& path, int width, int height, const unsigned char* rgb){
	// raw scanlines, top-down, each prefixed by filter type 0 (none)
	const size_t stride = (size_t)width * 3;
	std::vector<unsigned char> raw((stride + 1) * height);
	for (int y = 0; y < height; y++){
		unsigned char* row = raw.data() + (stride + 1) * y;
		row[0] = 0;
		const unsigned char* src = rgb + stride * (height - 1 - y);   // flip vertically
		std::copy(src, src + stride, row + 1);
	}
	uLongf zlen = compressBound((uLong)raw.size());
	std::vector<unsigned char> z(zlen);
	if (compress2(z.data(), &zlen, raw.data(), (uLong)raw.size(), Z_BEST_COMPRESSION) != Z_OK)
		return false;
	z.resize(zlen);

	std::vector<unsigned char> png = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n' };
	std::vector<unsigned char> ihdr;
	put32(ihdr, (unsigned int)width);
	put32(ihdr, (unsigned int)height);
	ihdr.insert(ihdr.end(), { 8, 2, 0, 0, 0 });   // 8 bit, RGB, deflate, no filter set, no interlace
	putChunk(png, "IHDR", ihdr);
	putChunk(png, "IDAT", z);
	putChunk(png, "IEND", {});

	FILE* f = fopen(path.c_str(), "wb");
	if (!f) return false;
	const bool ok = fwrite(png.data(), 1, png.size(), f) == png.size();
	return (fclose(f) == 0) && ok;
}

bool saveFramebufferPNG(const std::string& path, int width, int height){
	std::vector<unsigned char> rgb((size_t)width * height * 3);
	glReadBuffer(GL_BACK);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
	return writePNG(path, width, height, rgb.data());
}
