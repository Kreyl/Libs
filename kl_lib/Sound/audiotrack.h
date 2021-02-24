#pragma once

#include <cstddef>
#include <cstdint>

#include "wavreader.h"

#define HAS_COSINE_TABLE

class AudioTrack
{
public:
    typedef WavReader::Mode Mode;

    enum class Fade
    {
        None,
        LinearIn,
        LinearOut,
#ifdef HAS_COSINE_TABLE
        CosineIn,
        CosineOut,
        SCurveIn,
        SCurveOut,
#endif
    };

    static const uint8_t UNIT_LEVEL_SHIFT = 12;
    static const uint16_t UNIT_LEVEL = 1 << UNIT_LEVEL_SHIFT;

    static const uint8_t MAX_LEVEL_SHIFT = 14;
    static const uint16_t MAX_LEVEL = 1 << MAX_LEVEL_SHIFT;

    static const unsigned int MAX_TRACK_CHANNELS = 2;

public:
    AudioTrack();

    AudioTrack(WavReader::TellCallback tell_callback,
               WavReader::SeekCallback seek_callback,
               WavReader::ReadCallback read_callback,
               unsigned int channels);

    void init(WavReader::TellCallback tell_callback,
              WavReader::SeekCallback seek_callback,
              WavReader::ReadCallback read_callback,
              unsigned int channels);

    bool start(void *file,
               Mode mode,
               bool preload = true,
               uint16_t level = UNIT_LEVEL,
               Fade fade_mode = Fade::None,
               uint16_t fade_length_ms = 0);

    void fade(uint16_t level,
              Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    void stop(Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    void rewind(bool preload = true);

    size_t play(int16_t *buffer, size_t frames);

    bool running()
    {
        return running_;
    }

    void *playingNow()
    {
        return running_ ? file_ : nullptr;
    }

    Mode mode()
    {
        return reader_.mode();
    }

    unsigned int channels()
    {
        return channels_;
    }

    unsigned long samplingRate()
    {
        return reader_.samplingRate();
    }

private:
    bool initialized_;

    unsigned int channels_;
    unsigned int upmixing_;

    uint16_t frames_per_ms_;

    uint16_t level_;

    Fade fade_mode_;

    uint16_t fade_length_ms_;
    uint32_t fade_length_;
    uint32_t fade_progress_;

    uint16_t initial_level_;
    uint16_t final_level_;

    WavReader reader_;
    void *file_;

    bool running_;
    bool stopping_;
};
