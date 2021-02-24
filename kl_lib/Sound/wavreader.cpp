#include "wavreader.h"

#include <cstring>

#ifdef __GLIBC__
#include <endian.h>
#else
#define swap16(x) ((uint16_t)((((x) & 0xff00) >> 8) | \
                              (((x) & 0xff) << 8)))
#define swap32(x) ((uint32_t)((((x) & 0xff000000) >> 24) | \
                              (((x) & 0xff0000) >> 8) | \
                              (((x) & 0xff00) << 8) | \
                              (((x) & 0xff) << 24)))

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define le16toh(x) ((uint16_t)(x))
#define le32toh(x) ((uint32_t)(x))
#else
#define le16toh(x) ((uint16_t)(swap16(x)))
#define le32toh(x) ((uint32_t)(swap32(x)))
#endif
#endif

WavReader::WavReader(TellCallback tell_callback,
                     SeekCallback seek_callback,
                     ReadCallback read_callback)
    : opened_(false),
      tell_callback_(tell_callback),
      seek_callback_(seek_callback),
      read_callback_(read_callback)
{
}

void WavReader::init(WavReader::TellCallback tell_callback,
                     WavReader::SeekCallback seek_callback,
                     WavReader::ReadCallback read_callback)
{
    tell_callback_ = tell_callback;
    seek_callback_ = seek_callback;
    read_callback_ = read_callback;
}

bool WavReader::open(void *file_context,
                     WavReader::Mode mode,
                     bool preload)
{
    char chunk_id[4];
    uint32_t chunk_size;
    size_t next_chunk_offset;

    opened_ = false;

    file_context_ = file_context;

    mode_ = mode;

    next_chunk_offset = 0;

    while (true) {
        if (!seek(next_chunk_offset)) {
            return false;
        }

        if (!readCharBuffer(chunk_id, sizeof(chunk_id))) {
            return false;
        }

        if (!readU32(&chunk_size)) {
            return false;
        }

        if (memcmp(chunk_id, "RIFF", sizeof(chunk_id)) == 0) {
            break;
        }

        next_chunk_offset = tell() + chunk_size;
        if ((next_chunk_offset & 1) != 0) {
            next_chunk_offset++;
        }
    }

    file_size_ = chunk_size;

    char riff_type[4];

    if (!readCharBuffer(riff_type, sizeof(riff_type))) {
        return false;
    }

    if (memcmp(riff_type, "WAVE", sizeof(riff_type)) != 0) {
        return false;
    }

    while (true) {
        if (!readCharBuffer(chunk_id, sizeof(chunk_id))) {
            return false;
        }

        if (!readU32(&chunk_size)) {
            return false;
        }

        next_chunk_offset = tell() + chunk_size;
        if ((next_chunk_offset & 1) != 0) {
            next_chunk_offset++;
        }

        if (memcmp(chunk_id, "fmt ", sizeof(chunk_id)) == 0) {
            break;
        }

        if (!seek(next_chunk_offset)) {
            return false;
        }
    }

    uint16_t format;

    if (!readU16(&format)) {
        return false;
    }

    switch (format) {
    case static_cast<uint16_t>(Format::Pcm):
        format_ = Format::Pcm;
        break;
    default:
        return false;
    }

    uint16_t channels;

    if (!readU16(&channels)) {
        return false;
    }

    if (channels > MAX_CHANNELS) {
        return false;
    }

    channels_ = channels;

    uint32_t sampling_rate;

    if (!readU32(&sampling_rate)) {
        return false;
    }

    sampling_rate_ = sampling_rate;

    uint32_t bytes_per_second;

    if (!readU32(&bytes_per_second)) {
        return false;
    }

    bytes_per_second_ = bytes_per_second;

    uint16_t block_alignment;

    if (!readU16(&block_alignment)) {
        return false;
    }

    block_alignment_ = block_alignment;

    if (format_ == Format::Pcm) {
        uint16_t bits_per_sample;

        if (!readU16(&bits_per_sample)) {
            return false;
        }

        bits_per_sample_ = bits_per_sample;

        frame_size_ = block_alignment_;

        if (frame_size_ > MAX_FRAME_SIZE) {
            return false;
        }

        channel_size_ = frame_size_ / channels_;
    } else {
        bits_per_sample_ = 0;

        frame_size_ = 0;

        channel_size_ = 0;
    }

    while (true) {
        if (!seek(next_chunk_offset)) {
            return false;
        }

        if (!readCharBuffer(chunk_id, sizeof(chunk_id))) {
            return false;
        }

        if (!readU32(&chunk_size)) {
            return false;
        }

        if (memcmp(chunk_id, "data", sizeof(chunk_id)) == 0) {
            initial_data_chunk_offset_ = next_chunk_offset;
            if ((initial_data_chunk_offset_ & 1) != 0) {
                initial_data_chunk_offset_++;
            }

            final_data_chunk_offset_ = tell() + chunk_size;
            if ((final_data_chunk_offset_ & 1) != 0) {
                final_data_chunk_offset_++;
            }

            break;
        }

        next_chunk_offset = tell() + chunk_size;
        if ((next_chunk_offset & 1) != 0) {
            next_chunk_offset++;
        }

        if (memcmp(chunk_id, "LIST", sizeof(chunk_id)) == 0) {
            char list_type[4];

            if (!readCharBuffer(list_type, sizeof(list_type))) {
                return false;
            }

            if (memcmp(list_type, "wavl", sizeof(list_type)) != 0) {
                continue;
            }

            initial_data_chunk_offset_ = tell();
            if ((initial_data_chunk_offset_ & 1) != 0) {
                initial_data_chunk_offset_++;
            }

            final_data_chunk_offset_ = tell() + chunk_size;
            if ((final_data_chunk_offset_ & 1) != 0) {
                final_data_chunk_offset_++;
            }

            break;
        }
    }

    opened_ = true;

    rewind(preload);

    return true;
}

void WavReader::close()
{
    opened_ = false;
}

void WavReader::rewind(bool preload)
{
    next_data_chunk_offset_ = initial_data_chunk_offset_;
    current_data_chunk_frames_ = 0;

    memset(frame_buffer_, 0, MAX_FRAME_SIZE);
    current_frame_ = frame_buffer_;
    next_frame_ = frame_buffer_;
    prefetched_frames_ = 0;

    if (preload) {
        if (prepareCurrentChunk()) {
            if (!silence_) {
                prefetchNextFrames();
            }
        }
    }
}

size_t WavReader::decodeToI16(int16_t *buffer, size_t frames, unsigned int upmixing)
{
    if (!opened_) {
        return 0;
    }

    int16_t *frame_pointer = buffer;
    size_t processed_frames = 0;

    while (processed_frames < frames) {
        size_t decoded_frames = decodeNextFrames(frames - processed_frames);
        if (decoded_frames == 0) {
            break;
        }

        if (channel_size_ == 1) {
            uint8_t *sample_pointer = current_frame_;

            for (size_t frame_index = 0; frame_index < decoded_frames; frame_index++) {
                for (unsigned int channel = 0; channel < channels_; channel++) {
                    int16_t sample;
                    sample = static_cast<int16_t>(*sample_pointer) - 128;
                    sample = static_cast<int16_t>(sample << 8);
                    sample_pointer++;

                    for (unsigned int copy = 0; copy < upmixing; copy++) {
                        *frame_pointer = sample;
                        frame_pointer++;
                    }
                }
            }
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        } else if ((channel_size_ == 2) && (upmixing == 1)) {
            size_t samples = decoded_frames * channels_;
            memcpy(frame_pointer, current_frame_, samples * 2);
            frame_pointer += samples;
#endif
        } else {
            uint8_t *sample_pointer = current_frame_ + channel_size_ - 2;

            for (size_t frame_index = 0; frame_index < decoded_frames; frame_index++) {
                for (unsigned int channel = 0; channel < channels_; channel++) {
                    int16_t sample;
                    memcpy(&sample, sample_pointer, 2);
                    sample = le16toh(sample);
                    sample_pointer += channel_size_;

                    for (unsigned int copy = 0; copy < upmixing; copy++) {
                        *frame_pointer = sample;
                        frame_pointer++;
                    }
                }
            }
        }

        processed_frames += decoded_frames;
    }

    return processed_frames;
}

inline size_t WavReader::tell()
{
    return tell_callback_(file_context_);
}

inline bool WavReader::seek(size_t offset)
{
    return seek_callback_(file_context_, offset);
}

inline size_t WavReader::read(uint8_t *buffer, size_t length)
{
    return read_callback_(file_context_, buffer, length);
}

inline bool WavReader::readU16(uint16_t *value)
{
    if (read(reinterpret_cast<uint8_t *>(value),
             sizeof(uint16_t)) < sizeof(uint16_t)) {
        return false;
    }

    *value = le16toh(*value);

    return true;
}

inline bool WavReader::readU32(uint32_t *value)
{
    if (read(reinterpret_cast<uint8_t *>(value),
             sizeof(uint32_t)) < sizeof(uint32_t)) {
        return false;
    }

    *value = le32toh(*value);

    return true;
}

inline bool WavReader::readCharBuffer(char *buffer, size_t length)
{
    if (read(reinterpret_cast<uint8_t *>(buffer),
             length) < length) {
        return false;
    }

    return true;
}

inline size_t WavReader::decodeNextFrames(size_t frames)
{
    switch (format_) {
    case Format::Pcm:
        return decodeNextPcmFrames(frames);
    }

    return 0;
}

size_t WavReader::decodeNextPcmFrames(size_t frames)
{
    if (!prepareCurrentChunk()) {
        return 0;
    }

    if (!silence_) {
        if (prefetched_frames_ == 0) {
            if (prefetchNextFrames() < 1) {
                return 0;
            }

            next_frame_ = frame_buffer_;
        }

        current_frame_ = next_frame_;

        if (frames > prefetched_frames_) {
            frames = prefetched_frames_;
        }

        next_frame_ = current_frame_ + frame_size_ * frames;
        prefetched_frames_ -= frames;
    } else {
        frames = 1;
    }

    current_data_chunk_frames_ -= frames;

    return frames;
}

bool WavReader::prepareCurrentChunk()
{
    if (current_data_chunk_frames_ == 0) {
        if (next_data_chunk_offset_ == final_data_chunk_offset_) {
            if (mode_ == Mode::Continuous) {
                rewind(false);
            } else {
                return false;
            }
        }

        if (!seek(next_data_chunk_offset_)) {
            return false;
        }

        char chunk_id[4];
        uint32_t chunk_size;

        if (!readCharBuffer(chunk_id, sizeof(chunk_id))) {
            return false;
        }

        if (memcmp(chunk_id, "data", sizeof(chunk_id)) == 0) {
            silence_ = false;
        } else if (memcmp(chunk_id, "slnt", sizeof(chunk_id)) == 0) {
            silence_ = true;
        } else {
            return false;
        }

        if (!readU32(&chunk_size)) {
            return false;
        }

        if (!silence_) {
            current_data_chunk_frames_ = chunk_size / frame_size_;
        } else {
            uint32_t silent_frames;

            if (!readU32(&silent_frames)) {
                return false;
            }

            current_data_chunk_frames_ = silent_frames;
        }

        next_data_chunk_offset_ = tell() + chunk_size;
        if ((next_data_chunk_offset_ & 1) != 0) {
            next_data_chunk_offset_++;
        }
    }

    return true;
}

size_t WavReader::prefetchNextFrames()
{
    size_t frames_to_read = WAVREADER_BUFFER_SIZE / frame_size_;
    if (frames_to_read > current_data_chunk_frames_) {
        frames_to_read = current_data_chunk_frames_;
    }

    size_t bytes_to_read = frames_to_read * frame_size_;
    size_t read_bytes = read(frame_buffer_, bytes_to_read);

    size_t read_frames = read_bytes / frame_size_;

    prefetched_frames_ = read_frames;

    return read_frames;
}
