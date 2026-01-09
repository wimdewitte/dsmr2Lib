#ifndef DSMR_INCLUDE_READER_H
#define DSMR_INCLUDE_READER_H

#include <Arduino.h>
#include "crc16.h"
#include "parser2.h"

// Define a buffer size appropriate for your meter's telegram length.
#define P1_BUFFER_SIZE 1536

namespace dsmr
{
    class P1Reader
    {
    public:
        P1Reader(Stream *stream, uint8_t req_pin, bool checksum = true)
            : stream(stream), req_pin(req_pin), once(false), state(State::DISABLED_STATE), buffer_idx(0), _available(false)
        {
            if (req_pin > -1) {
                pinMode(req_pin, OUTPUT);
                digitalWrite(req_pin, LOW);
            }
            this->checksum = checksum;
            buffer[0] = '\0'; // Initialize empty string
        }

        void doChecksum(bool checksum)
        {
            this->checksum = checksum;
        }

        void enable(bool once)
        {
            if (req_pin > -1)
                digitalWrite(this->req_pin, HIGH);
            this->state = State::WAITING_STATE;
            this->once = once;
        }

        void disable()
        {
            if (req_pin > -1)
                digitalWrite(this->req_pin, LOW);
            this->state = State::DISABLED_STATE;
            if (!this->_available)
            {
                this->clear();
            }
            while (this->stream->read() >= 0)
                ; // Flush serial
        }

        bool available()
        {
            return this->_available;
        }

        uint16_t GetCRC()
        {
            return this->crc;
        }

        bool loop()
        {
            while (true)
            {
                if (state == State::CHECKSUM_STATE)
                {
                    if (this->checksum)
                    {
                        if ((size_t)this->stream->available() < CrcParser::CRC_LEN)
                            return false;
                    }

                    char crc_buf[CrcParser::CRC_LEN];
                    for (uint8_t i = 0; i < CrcParser::CRC_LEN; ++i)
                    {
                        crc_buf[i] = this->stream->read();
                    }

                    ParseResult<uint16_t> crc_res = CrcParser::parse(crc_buf, crc_buf + CrcParser::CRC_LEN);

                    state = State::WAITING_STATE;

                    if (this->checksum)
                    {
                        if (!crc_res.err && crc_res.result == this->crc)
                        {
                            this->_available = true;
                        }
                    }
                    else
                    {
                        this->_available = true;
                    }

                    if (once)
                        this->disable();
                    return true;
                }
                else
                {
                    int c = this->stream->read();
                    if (c < 0)
                        return false;

                    switch (this->state)
                    {
                    case State::DISABLED_STATE:
                        break;

                    case State::WAITING_STATE:
                        if (c == '/')
                        {
                            this->state = State::READING_STATE;
                            this->crc = _crc16_update(0, c);
                            this->clear(); // Resets index to 0
                        }
                        break;

                    case State::READING_STATE:
                        this->crc = _crc16_update(this->crc, c);
                        if (c == '!')
                        {
                            this->state = State::CHECKSUM_STATE;
                            // buffer[buffer_idx] = '\0'; // Null terminate the data gathered so far
                        }
                        else
                        {
                            // Check for overflow before adding to buffer
                            if (buffer_idx < (P1_BUFFER_SIZE - 1))
                            {
                                buffer[buffer_idx++] = (char)c;
                            }
                            else
                            {
                                // Buffer overflow: Reset and wait for next message
                                Serial.println("P1 buffer overflow");
                                this->state = State::WAITING_STATE;
                                this->clear();
                            }
                        }
                        break;

                    default:
                        break;
                    }
                }
                yield();
            }
            return false;
        }

        // Returns a pointer to the internal fixed array
        const char *raw()
        {
            return buffer;
        }

        template <typename... Ts>
        bool parse(ParsedData<Ts...> *data, String *err)
        {
            const char *str = buffer;
            const char *end = buffer + buffer_idx;
            ParseResult<void> res = P1Parser::parse_data(data, str, end);

            if (res.err && err)
                *err = res.fullError(str, end);

            this->clear();
            return res.err == NULL;
        }

        void clear()
        {
            buffer_idx = 0;
            buffer[0] = '\0';
            _available = false;
        }

        void ChangeStream(Stream *new_stream)
        {
            stream = new_stream;
            this->clear();
        }

    protected:
        Stream *stream;
        uint8_t req_pin;
        enum class State : uint8_t
        {
            DISABLED_STATE,
            WAITING_STATE,
            READING_STATE,
            CHECKSUM_STATE,
        };
        bool _available;
        bool once, checksum;
        State state;

        // Fixed Buffer Implementation
        char buffer[P1_BUFFER_SIZE];
        uint16_t buffer_idx;
        uint16_t crc;
    };

} // namespace dsmr

#endif