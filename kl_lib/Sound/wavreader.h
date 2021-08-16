#pragma once

#include <cstddef>
#include <cstdint>

#include "audioreader.h"

#define HAS_IEEE_FLOAT

#ifndef WAVREADER_BUFFER_SIZE
#define WAVREADER_BUFFER_SIZE 2048
#endif

class WavReader : public AudioReader
{
public:
    enum class Format : unsigned int
    {
        Unknown = 0,
        Pcm = 1,
#ifdef HAS_IEEE_FLOAT
        IeeeFloat = 3,
#endif
    };

    static const unsigned int MAX_CHANNELS = 2;

    static const unsigned int MAX_FRAME_SIZE = 16;

public:
    WavReader(TellCallback tell_callback,
              SeekCallback seek_callback,
              ReadCallback read_callback);

    bool open(void *file,
              Mode mode = Mode::Single,
              bool preload = true) override;

    void close() override;

    void rewind(bool preload = true) override;

    size_t decodeToI16(int16_t *buffer, size_t frames, unsigned int upmixing = 1) override;

    Format format()
    {
        return format_;
    }

    unsigned long bytesPerSecond()
    {
        return bytes_per_second_;
    }

    unsigned int bitsPerSample()
    {
        return bits_per_sample_;
    }

    size_t blockAlignment()
    {
        return block_alignment_;
    }

private:
    inline size_t tell();
    inline bool seek(size_t offset);
    inline size_t read(uint8_t *buffer, size_t length);

    inline bool readU16(uint16_t *value);
    inline bool readU32(uint32_t *value);
    inline bool readCharBuffer(char *buffer, size_t length);

    inline size_t decodeNextFrames(size_t frames);

    size_t decodeNextPcmFrames(size_t frames);
#ifdef HAS_IEEE_FLOAT
    size_t decodeNextIeeeFloatFrames(size_t frames);
#endif

    size_t retrieveNextFrames(size_t frames);

    bool prepareCurrentChunk();
    size_t prefetchNextFrames();

private:
    size_t file_size_;

    Format format_;
    unsigned long bytes_per_second_;
    unsigned int bits_per_sample_;
    size_t block_alignment_;
    size_t frame_size_;
    size_t channel_size_;

    size_t initial_data_chunk_offset_;
    size_t final_data_chunk_offset_;
    size_t next_data_chunk_offset_;
    size_t current_data_chunk_frames_;

    alignas(4) uint8_t frame_buffer_[WAVREADER_BUFFER_SIZE];
    size_t prefetched_frames_;
    uint8_t *current_frame_;
    uint8_t *next_frame_;

    bool silence_;
};
