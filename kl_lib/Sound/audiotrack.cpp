#include "audiotrack.h"

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

#ifdef HAS_COSINE_TABLE
#include "cosine.h"
#endif

AudioTrack::AudioTrack(unsigned int channels)
    : readers_(),
      reader_(nullptr),
      file_(nullptr),
      channels_(channels),
      upmixing_(1),
      frames_per_ms_(0),
      level_(UNIT_LEVEL),
      fade_mode_(Fade::None),
      fade_length_ms_(0),
      fade_length_(0),
      fade_progress_(0),
      initial_level_(0),
      final_level_(0),
      running_(false),
      stopping_(false)
{
}

bool AudioTrack::addReader(AudioReader *reader)
{
    for (int slot = 0; slot < READER_SLOTS; slot++) {
        if (!readers_[slot]) {
            readers_[slot] = reader;
            return true;
        }
    }

    return false;
}

bool AudioTrack::start(void *file,
                       Mode mode,
                       bool preload,
                       uint16_t level,
                       Fade fade_mode,
                       uint16_t fade_length_ms)
{
    running_ = false;
    stopping_ = false;

    reader_ = nullptr;

    for (int slot = 0; slot < READER_SLOTS; slot++) {
        if (readers_[slot] && readers_[slot]->open(file, mode, preload)) {
            reader_ = readers_[slot];
            break;
        }
    }

    if (!reader_) {
        return false;
    }

    file_ = file;

    if (reader_->channels() > MAX_TRACK_CHANNELS) {
        reader_->close();
        return false;
    }

    if (reader_->channels() != channels_) {
        if (channels_ % reader_->channels() != 0) {
            reader_->close();
            return false;
        }

        upmixing_ = channels_ / reader_->channels();
    } else {
        upmixing_ = 1;
    }

    // Not exactly precise, but good for overflow-free conversions
    frames_per_ms_ = static_cast<uint16_t>(reader_->samplingRate() / 1000);

    level_ = 0;

    running_ = true;

    fade(level, fade_mode, fade_length_ms);

    return true;
}

void AudioTrack::fade(uint16_t level,
                      Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (!reader_) {
        return;
    }

    if (!running_) {
        return;
    }

    if (level > MAX_LEVEL) {
        level = MAX_LEVEL;
    }

    fade_mode_ = fade_mode;

    if (fade_mode_ != Fade::None) {
        fade_length_ms_ = fade_length_ms;
        fade_length_ = fade_length_ms * frames_per_ms_;
        fade_progress_ = 0;

        initial_level_ = level_;
        final_level_ = level;
    } else {
        fade_length_ms_ = 0;
        fade_length_ = 0;
        fade_progress_ = 0;

        initial_level_ = level;
        level_ = level;
        final_level_ = level;
    }
}

void AudioTrack::stop(Fade fade_mode,
                      uint16_t fade_length_ms)
{
    if (!reader_) {
        return;
    }

    fade(0, fade_mode, fade_length_ms);

    if (fade_mode_ != Fade::None) {
        stopping_ = true;
    } else {
        reader_->close();

        stopping_ = false;
        running_ = false;
    }
}

void AudioTrack::rewind(bool preload)
{
    if (!reader_) {
        return;
    }

    if (!running_) {
        return;
    }

    reader_->rewind(preload);
}

size_t AudioTrack::play(int16_t *buffer, size_t frames)
{
    if (!reader_) {
        return 0;
    }

    if (!running_) {
        return 0;
    }

    frames = reader_->decodeToI16(buffer, frames, upmixing_);
    if (frames < 1) {
        stop(Fade::None, 0);
        return frames;
    }

    for (size_t frame_index = 0; frame_index < frames; frame_index++) {
        if (level_ != UNIT_LEVEL) {
            for (unsigned int channel = 0; channel < channels_; channel++) {
                size_t offset = channels_ * frame_index + channel;
                int32_t sample = (buffer[offset] * level_) / UNIT_LEVEL;
                buffer[offset] = saturate(sample);
            }
        }

        if (fade_mode_ != Fade::None) {
            if (fade_progress_ == fade_length_) {
                if (stopping_) {
                    stop(Fade::None, 0);
                    return frame_index;
                } else {
                    fade(final_level_, Fade::None, 0);
                }
            }

            fade_progress_++;

            int32_t level_offset = static_cast<int32_t>(final_level_) - static_cast<int32_t>(initial_level_);
            uint16_t fade_progress_ms;

            switch (fade_mode_) {
            case Fade::LinearIn:
            case Fade::LinearOut:
                fade_progress_ms = static_cast<uint16_t>(fade_progress_ / frames_per_ms_);
                level_offset *= fade_progress_ms;
                level_offset /= fade_length_ms_;
                level_ = initial_level_ + static_cast<uint16_t>(level_offset);
                break;
#ifdef HAS_COSINE_TABLE
            case Fade::CosineIn:
                fade_progress_ms = static_cast<uint16_t>(fade_progress_ / frames_per_ms_);
                level_offset *= cosineFromZeroToHalfPi(fade_length_ms_ - fade_progress_ms, fade_length_ms_);
                level_offset /= 32768;
                level_ = initial_level_ + static_cast<uint16_t>(level_offset);
                break;
            case Fade::CosineOut:
                fade_progress_ms = static_cast<uint16_t>(fade_progress_ / frames_per_ms_);
                level_offset *= 32768 - cosineFromZeroToHalfPi(fade_progress_ms, fade_length_ms_);
                level_offset /= 32768;
                level_ = initial_level_ + static_cast<uint16_t>(level_offset);
                break;
            case Fade::SCurveIn:
            case Fade::SCurveOut:
                fade_progress_ms = static_cast<uint16_t>(fade_progress_ / frames_per_ms_);
                level_offset *= 32768 - cosineFromZeroToHalfPi(fade_progress_ms * 2, fade_length_ms_);
                level_offset /= 65536;
                level_ = initial_level_ + static_cast<uint16_t>(level_offset);
                break;
#endif
            case Fade::None:
                break;
            }
        }
    }

    return frames;
}
