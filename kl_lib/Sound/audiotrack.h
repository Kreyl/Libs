#pragma once

#include <cstddef>
#include <cstdint>

#include "audioreader.h"

class AudioTrack
{
public:
    typedef AudioReader::Mode Mode;

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

    static const int READER_SLOTS = 2;

    static const uint8_t UNIT_LEVEL_SHIFT = 12;
    static const uint16_t UNIT_LEVEL = 1 << UNIT_LEVEL_SHIFT;

    static const uint8_t MAX_LEVEL_SHIFT = 14;
    static const uint16_t MAX_LEVEL = 1 << MAX_LEVEL_SHIFT;

    static const unsigned int MAX_TRACK_CHANNELS = 2;

public:
    AudioTrack(unsigned int channels);

    bool addReader(AudioReader *reader);

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
        return reader_ ? reader_->mode() : Mode::Single;
    }

    unsigned long samplingRate()
    {
        return reader_ ? reader_->samplingRate() : 0;
    }

    unsigned int channels()
    {
        return channels_;
    }

private:
    AudioReader *readers_[READER_SLOTS];
    AudioReader *reader_;
    void *file_;

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

    bool running_;
    bool stopping_;
};
