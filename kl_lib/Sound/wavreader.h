#pragma once

#include <cstddef>
#include <cstdint>

#ifndef WAVREADER_BUFFER_SIZE
#define WAVREADER_BUFFER_SIZE 2048
#endif

class WavReader
{
public:
    typedef size_t (*TellCallback)(void *file_context);
    typedef bool (*SeekCallback)(void *file_context, size_t offset);
    typedef size_t (*ReadCallback)(void *file_context, uint8_t *buffer, size_t length);

    enum class Mode
    {
        Single,
        Continuous
    };

    enum class Format : unsigned int
    {
        Pcm = 1
    };

    static const unsigned int MAX_CHANNELS = 2;

    static const unsigned int MAX_FRAME_SIZE = 16;

public:
    WavReader(TellCallback tell_callback,
              SeekCallback seek_callback,
              ReadCallback read_callback);

    void init(TellCallback tell_callback,
              SeekCallback seek_callback,
              ReadCallback read_callback);

    bool open(void *file_context,
              Mode mode = Mode::Single,
              bool preload = true);

    void close();

    void rewind(bool preload = true);

    size_t decodeToI16(int16_t *buffer, size_t frames, unsigned int upmixing = 1);

    bool opened()
    {
        return opened_;
    }

    Mode mode()
    {
        return mode_;
    }

    Format format()
    {
        return format_;
    }

    unsigned int channels()
    {
        return channels_;
    }

    unsigned long samplingRate()
    {
        return sampling_rate_;
    }

    unsigned long bytesPerSecond()
    {
        return bytes_per_second_;
    }

    size_t blockAlignment()
    {
        return block_alignment_;
    }

    unsigned int bitsPerSample()
    {
        return bits_per_sample_;
    }

    size_t frameSize()
    {
        return frame_size_;
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

    bool prepareCurrentChunk();
    size_t prefetchNextFrames();

private:
    bool opened_;

    Mode mode_;

    void *file_context_;

    TellCallback tell_callback_;
    SeekCallback seek_callback_;
    ReadCallback read_callback_;

    size_t file_size_;

    Format format_;
    unsigned int channels_;
    unsigned long sampling_rate_;
    unsigned long bytes_per_second_;
    size_t block_alignment_;
    unsigned int bits_per_sample_;
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
