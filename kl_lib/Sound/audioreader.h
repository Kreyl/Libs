#pragma once

#include <cstddef>
#include <cstdint>

class AudioReader
{
public:
    typedef size_t (*TellCallback)(void *file);
    typedef bool (*SeekCallback)(void *file, size_t offset);
    typedef size_t (*ReadCallback)(void *file, uint8_t *buffer, size_t length);

    enum class Mode
    {
        Single,
        Continuous,
    };

public:
    AudioReader(TellCallback tell_callback,
                SeekCallback seek_callback,
                ReadCallback read_callback)
        : opened_(false),
          file_(nullptr),
          mode_(Mode::Single),
          tell_callback_(tell_callback),
          seek_callback_(seek_callback),
          read_callback_(read_callback),
          sampling_rate_(0),
          channels_(0)
    {
    }

    virtual bool open(void *file,
                      Mode mode = Mode::Single,
                      bool preload = true) = 0;

    virtual void close() = 0;

    virtual void rewind(bool preload = true) = 0;

    virtual size_t decodeToI16(int16_t *buffer, size_t frames, unsigned int upmixing = 1) = 0;

    bool opened()
    {
        return opened_;
    }

    Mode mode()
    {
        return mode_;
    }

    unsigned long samplingRate()
    {
        return sampling_rate_;
    }

    unsigned int channels()
    {
        return channels_;
    }

protected:
    bool opened_;

    void *file_;

    Mode mode_;

    TellCallback tell_callback_;
    SeekCallback seek_callback_;
    ReadCallback read_callback_;

    unsigned long sampling_rate_;
    unsigned int channels_;
};
