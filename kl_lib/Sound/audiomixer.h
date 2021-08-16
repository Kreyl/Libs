#pragma once

#include <cstddef>
#include <cstdint>

#ifndef AUDIOMIXER_BUFFER_SIZE
#define AUDIOMIXER_BUFFER_SIZE 4096
#endif

#include "audiotrack.h"

class AudioMixer
{
public:
    typedef void (*TrackEndCallback)(int slot);

    typedef AudioTrack Track;

    typedef AudioTrack::Mode Mode;

    typedef AudioTrack::Fade Fade;

    static const int TRACK_SLOTS = 4;

    static const uint16_t UNIT_LEVEL = AudioTrack::UNIT_LEVEL;

    static const uint16_t MAX_LEVEL = AudioTrack::MAX_LEVEL;

    static const size_t AUDIOMIXER_BUFFER_LENGTH = AUDIOMIXER_BUFFER_SIZE / 4;

public:
    AudioMixer(TrackEndCallback track_end_callback,
               unsigned int channels);

    bool addTrack(Track *track);

    void scale(uint16_t level);

    int start(void *file,
              Mode mode,
              bool preload = true,
              uint16_t level = UNIT_LEVEL,
              Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    bool start(int slot,
               void *file,
               Mode mode,
               bool preload = true,
               uint16_t level = UNIT_LEVEL,
               Fade fade_mode = Fade::None,
               uint16_t fade_length_ms = 0);

    void fade(uint16_t level,
              Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    void fade(int slot,
              uint16_t level,
              Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    void stop(Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    void stop(int slot,
              Fade fade_mode = Fade::None,
              uint16_t fade_length_ms = 0);

    void clear();

    size_t play(int16_t *buffer, size_t frames);

    unsigned long samplingRate()
    {
        return sampling_rate_;
    }

    unsigned int channels()
    {
        return channels_;
    }

private:
    Track *tracks_[TRACK_SLOTS];

    int32_t sample_buffer_[AUDIOMIXER_BUFFER_LENGTH];

    TrackEndCallback track_end_callback_;

    unsigned long sampling_rate_;
    unsigned int channels_;

    uint16_t level_;
};
