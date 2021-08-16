#include "audiomixer.h"

#include <cstring>
#include <limits>

#ifdef __ARM_ACLE
#include <arm_acle.h>
#endif

static inline int16_t saturate(int32_t value)
{
#if defined(__ARM_ACLE) && defined(__ARM_FEATURE_SAT)
    return static_cast<int16_t>(__ssat(value, 16));
#else
    if (value > std::numeric_limits<int16_t>::max()) {
        value = std::numeric_limits<int16_t>::max();
    } else if (value < std::numeric_limits<int16_t>::min()) {
        value = std::numeric_limits<int16_t>::min();
    }

    return static_cast<int16_t>(value);
#endif
}

AudioMixer::AudioMixer(TrackEndCallback track_end_callback,
                       unsigned int channels)
    : tracks_(),
      sample_buffer_(),
      track_end_callback_(track_end_callback),
      sampling_rate_(0),
      channels_(channels),
      level_(UNIT_LEVEL)
{
}

bool AudioMixer::addTrack(Track *track)
{
    for (int slot = 0; slot < TRACK_SLOTS; slot++) {
        if (!tracks_[slot]) {
            tracks_[slot] = track;
            return true;
        }
    }

    return false;
}

void AudioMixer::scale(uint16_t level)
{
    if (level > MAX_LEVEL) {
        level = MAX_LEVEL;
    }

    level_ = level;
}

int AudioMixer::start(void *file,
                      Mode mode,
                      bool preload,
                      uint16_t level,
                      Fade fade_mode,
                      uint16_t fade_length_ms)
{
    for (int slot = 0; slot < TRACK_SLOTS; slot++) {
        if (tracks_[slot] && !tracks_[slot]->running()) {
            if (start(slot, file, mode, preload, level, fade_mode, fade_length_ms)) {
                return slot;
            }

            break;
        }
    }

    return -1;
}

bool AudioMixer::start(int slot,
                       void *file,
                       Mode mode,
                       bool preload,
                       uint16_t level,
                       Fade fade_mode,
                       uint16_t fade_length_ms)
{
    if (slot >= TRACK_SLOTS) {
        return false;
    }

    if (!tracks_[slot]) {
        return false;
    }

    if (tracks_[slot]->running()) {
        tracks_[slot]->stop();
    }

    if (!tracks_[slot]->start(file, mode, preload, level, fade_mode, fade_length_ms)) {
        return false;
    }

    if (tracks_[slot]->samplingRate() != sampling_rate_) {
        for (int other_index = 0; other_index < TRACK_SLOTS; other_index++) {
            if (other_index != slot) {
                stop(other_index);
            }
        }

        sampling_rate_ = tracks_[slot]->samplingRate();
    }

    return true;
}

void AudioMixer::fade(uint16_t level,
                      Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (level > MAX_LEVEL) {
        level = MAX_LEVEL;
    }

    for (int slot = 0; slot < TRACK_SLOTS; slot++) {
        fade(slot, level, fade_mode, fade_length_ms);
    }
}

void AudioMixer::fade(int slot,
                      uint16_t level,
                      Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (slot >= TRACK_SLOTS) {
        return;
    }

    if (!tracks_[slot]) {
        return;
    }

    if (tracks_[slot]->running()) {
        tracks_[slot]->fade(level, fade_mode, fade_length_ms);
    }
}

void AudioMixer::stop(Fade fade_mode,
                      uint16_t fade_length_ms)
{
    for (int slot = 0; slot < TRACK_SLOTS; slot++) {
        stop(slot, fade_mode, fade_length_ms);
    }
}

void AudioMixer::stop(int slot,
                      Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (slot >= TRACK_SLOTS) {
        return;
    }

    if (!tracks_[slot]) {
        return;
    }

    if (tracks_[slot]->running()) {
        tracks_[slot]->stop(fade_mode, fade_length_ms);
    }
}

void AudioMixer::clear()
{
    stop();
}

size_t AudioMixer::play(int16_t *buffer, size_t frames)
{
    size_t batch_size = AUDIOMIXER_BUFFER_SIZE;
    size_t batch_samples = AUDIOMIXER_BUFFER_LENGTH;
    size_t batch_frames = batch_samples / channels_;

    size_t remaining_frames = frames;

    while (remaining_frames > 0) {
        if (batch_frames > remaining_frames) {
            batch_frames = remaining_frames;
            batch_samples = batch_frames * channels_;
            batch_size = batch_samples * 4;
        }

        memset(sample_buffer_, 0, batch_size);

        for (int slot = 0; slot < TRACK_SLOTS; slot++) {
            if (tracks_[slot] && tracks_[slot]->running()) {
                size_t track_frames = tracks_[slot]->play(buffer, batch_frames);
                if (track_frames < 1) {
                    tracks_[slot]->stop();
                    track_end_callback_(slot);
                    continue;
                }

                for (size_t frame_index = 0; frame_index < track_frames; frame_index++) {
                    for (unsigned int channel = 0; channel < channels_; channel++) {
                        size_t offset = channels_ * frame_index + channel;
                        sample_buffer_[offset] += buffer[offset];
                    }
                }
            }
        }

        for (size_t frame_index = 0; frame_index < batch_frames; frame_index++) {
            for (unsigned int channel = 0; channel < channels_; channel++) {
                size_t offset = channels_ * frame_index + channel;
                buffer[offset] = saturate((sample_buffer_[offset] * level_) / UNIT_LEVEL);
            }
        }

        remaining_frames -= batch_frames;
        buffer += batch_samples;
    }

    return frames;
}
