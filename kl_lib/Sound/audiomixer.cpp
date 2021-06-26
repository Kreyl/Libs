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

AudioMixer::AudioMixer(WavReader::TellCallback tell_callback,
                       WavReader::SeekCallback seek_callback,
                       WavReader::ReadCallback read_callback,
                       TrackEndCallback track_end_callback,
                       unsigned int channels)
    : tracks_(),
      track_end_callback_(track_end_callback),
      sampling_rate_(0),
      channels_(channels),
      level_(UNIT_LEVEL)

{
    for (int track = 0; track < TRACKS; track++) {
        tracks_[track].init(tell_callback,
                            seek_callback,
                            read_callback,
                            channels);
    }
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
                      AudioMixer::Fade fade_mode,
                      uint16_t fade_length_ms)
{
    for (int track = 0; track < TRACKS; track++) {
        if (!tracks_[track].running()) {
            if (start(track, file, mode, preload, level, fade_mode, fade_length_ms)) {
                return track;
            }

            break;
        }
    }

    return -1;
}

bool AudioMixer::start(int track,
                       void *file,
                       Mode mode,
                       bool preload,
                       uint16_t level,
                       Fade fade_mode,
                       uint16_t fade_length_ms)
{
    if (tracks_[track].running()) {
        tracks_[track].stop();
    }

    if (!tracks_[track].start(file, mode, preload, level, fade_mode, fade_length_ms)) {
        return false;
    }

    if (tracks_[track].samplingRate() != sampling_rate_) {
        for (int other_track = 0; other_track < TRACKS; other_track++) {
            if (other_track != track) {
                stop(other_track);
            }
        }

        sampling_rate_ = tracks_[track].samplingRate();
    }

    return true;
}

void AudioMixer::fade(uint16_t level,
                      AudioMixer::Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (level > MAX_LEVEL) {
        level = MAX_LEVEL;
    }

    for (int track = 0; track < TRACKS; track++) {
        fade(track, level, fade_mode, fade_length_ms);
    }
}

void AudioMixer::fade(int track,
                      uint16_t level,
                      AudioMixer::Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (track >= TRACKS) {
        return;
    }

    if (tracks_[track].running()) {
        tracks_[track].fade(level, fade_mode, fade_length_ms);
    }
}

void AudioMixer::stop(Fade fade_mode,
                      uint16_t fade_length_ms)
{
    for (int track = 0; track < TRACKS; track++) {
        stop(track, fade_mode, fade_length_ms);
    }
}

void AudioMixer::stop(int track,
                      Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (track >= TRACKS) {
        return;
    }

    if (tracks_[track].running()) {
        tracks_[track].stop(fade_mode, fade_length_ms);
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

        for (int track = 0; track < TRACKS; track++) {
            if (tracks_[track].running()) {
                size_t track_frames = tracks_[track].play(buffer, batch_frames);
                if (track_frames < 1) {
                    tracks_[track].stop();
                    track_end_callback_(track);
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
