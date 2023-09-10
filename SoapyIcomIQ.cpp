/*
 * The MIT License (MIT)
 *
 * Copyright (C) 2023 Christoph Berg DF7CB <cb@df7cb.de>

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Logger.h>
#include <SoapySDR/Registry.hpp>
#include <ftd3xx.h>
#include <atomic>

#define CMD_OUT 0x02
#define CMD_IN  0x82
#define IQ_IN   0x84
#define TIMEOUT 100 // ms
#define DEFAULT_BUFFER_LENGTH 256 * 1024
#define DEFAULT_NUM_BUFFERS 15
#define BYTES_PER_SAMPLE 4

#define CMD_INDEX 4
#define SUBCMD_INDEX 5
#define DATA_INDEX 6

/***********************************************************************
 * Device interface
 **********************************************************************/
class IcomIQ : public SoapySDR::Device
{
private:
    std::string serial;
    FT_HANDLE handle;
    size_t numBuffers, bufferLength, asyncBuffs;
    size_t _currentHandle;
    size_t bufferedElems;
    std::atomic<bool> resetBuffer;
    std::string rxFormat;

public:
    IcomIQ(const SoapySDR::Kwargs &args);
    ~IcomIQ(void);

/*
    struct Buffer
    {
        unsigned long long tick;
        std::vector<signed char> data;
    };

    std::vector<Buffer> _buffs;
    size_t      _buf_head;
    size_t      _buf_tail;
    std::atomic<size_t> _buf_count;
    signed char *_currentBuff;
    */

    bool send_cmd(std::vector<uint8_t> cmd);
    std::vector<uint8_t> read_reply();
    int icom_cmd(std::vector<uint8_t> cmd, std::vector<uint8_t> &reply);

    // Identification API
    std::string getDriverKey(void) const;
    std::string getHardwareKey(void) const;
    SoapySDR::Kwargs getHardwareInfo(void) const;

    // Channels API
    size_t getNumChannels(const int dir) const;
    bool getFullDuplex(const int direction, const size_t channel) const;

    // Stream API
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;
    SoapySDR::Stream *setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels =
            std::vector<size_t>(), const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
    void closeStream(SoapySDR::Stream *stream);
    size_t getStreamMTU(SoapySDR::Stream *stream) const;
    int activateStream(
            SoapySDR::Stream *stream,
            const int flags = 0,
            const long long timeNs = 0,
            const size_t numElems = 0);
    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0);
    int readStream(
            SoapySDR::Stream *stream,
            void * const *buffs,
            const size_t numElems,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000);
    // /*******************************************************************
    //  * Direct buffer access API
    //  ******************************************************************/
    // size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);
    // int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);
    // int acquireReadBuffer(
    //     SoapySDR::Stream *stream,
    //     size_t &handle,
    //     const void **buffs,
    //     int &flags,
    //     long long &timeNs,
    //     const long timeoutUs = 100000);
    // void releaseReadBuffer(
    //     SoapySDR::Stream *stream,
    //     const size_t handle);
    // /*******************************************************************
    //  * Antenna API
    //  ******************************************************************/
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
    void setAntenna(const int direction, const size_t channel, const std::string &name);
    std::string getAntenna(const int direction, const size_t channel) const;
    // /*******************************************************************
    //  * Frontend corrections API
    //  ******************************************************************/
    // bool hasDCOffsetMode(const int direction, const size_t channel) const;
    // bool hasFrequencyCorrection(const int direction, const size_t channel) const;
    // void setFrequencyCorrection(const int direction, const size_t channel, const double value);
    // double getFrequencyCorrection(const int direction, const size_t channel) const;
    // /*******************************************************************
    //  * Gain API
    //  ******************************************************************/
    // std::vector<std::string> listGains(const int direction, const size_t channel) const;
    // bool hasGainMode(const int direction, const size_t channel) const;
    // void setGainMode(const int direction, const size_t channel, const bool automatic);
    // bool getGainMode(const int direction, const size_t channel) const;
    // void setGain(const int direction, const size_t channel, const double value);
    // void setGain(const int direction, const size_t channel, const std::string &name, const double value);
    // double getGain(const int direction, const size_t channel, const std::string &name) const;
    // SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;
    // /*******************************************************************
    //  * Frequency API
    //  ******************************************************************/
    void setFrequency(
            const int direction,
            const size_t channel,
            const std::string &name,
            const double frequency,
            const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
    double getFrequency(const int direction, const size_t channel, const std::string &name);
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;
    // SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;

    // Sample Rate API
    void setSampleRate(const int direction, const size_t channel, const double rate);
    double getSampleRate(const int direction, const size_t channel) const;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const;
    // SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;
    // void setBandwidth(const int direction, const size_t channel, const double bw);
    // double getBandwidth(const int direction, const size_t channel) const;
    // std::vector<double> listBandwidths(const int direction, const size_t channel) const;
    // SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const;
    // /*******************************************************************
    //  * Time API
    //  ******************************************************************/
    // std::vector<std::string> listTimeSources(void) const;
    // std::string getTimeSource(void) const;
    // bool hasHardwareTime(const std::string &what = "") const;
    // long long getHardwareTime(const std::string &what = "") const;
    // void setHardwareTime(const long long timeNs, const std::string &what = "");
    // /*******************************************************************
    //  * Utility
    //  ******************************************************************/
    // static std::string rtlTunerToString(rtlsdr_tuner tunerType);
    // static rtlsdr_tuner rtlStringToTuner(std::string tunerType);
    // static int getE4000Gain(int stage, int gain);
    // /*******************************************************************
    //  * Settings API
    //  ******************************************************************/
    // SoapySDR::ArgInfoList getSettingInfo(void) const;
    // void writeSetting(const std::string &key, const std::string &value);
    // std::string readSetting(const std::string &key) const;
};

IcomIQ::IcomIQ(const SoapySDR::Kwargs &args)
{
    SoapySDR_logf(SOAPY_SDR_INFO, "IcomIQ::IcomIQ");

    if (args.count("label") != 0) SoapySDR_logf(SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

    //if a serial is not present, then findIcomIQ had zero devices enumerated
    if (args.count("serial") == 0) throw std::runtime_error("No IcomIQ devices found!");

    serial = args.at("serial");

    FT_Create((PVOID) (serial.c_str()), FT_OPEN_BY_SERIAL_NUMBER, &handle);
    if (!handle) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "FT_Create failed: %s\n", serial.c_str());
        throw std::runtime_error("Failed to open FTDI device");
    }

    std::vector<uint8_t> cmd = {0x1a, 0x0b};
    std::vector<uint8_t> reply;
    icom_cmd(cmd, reply);
}

IcomIQ::~IcomIQ(void)
{
    SoapySDR_logf(SOAPY_SDR_INFO, "IcomIQ::~IcomIQ");

    FT_AbortPipe(handle, CMD_OUT);
    FT_AbortPipe(handle, CMD_IN);
    FT_AbortPipe(handle, IQ_IN);
    FT_Close(handle);
}

bool
IcomIQ::send_cmd(std::vector <uint8_t> cmd)
{
    uint8_t buf[32] = {0xfe, 0xfe, 0x98, 0xe0};
    DWORD buf_size = 4;
    int res;
    DWORD count;

    for (size_t i = 0; i < cmd.size(); i++) {
        buf[buf_size++] = cmd[i];
    }
    buf[buf_size++] = 0xfd;
    for (size_t i = buf_size; i % 4 > 0; i++) {
        buf[buf_size++] = 0xff;
    }

    for (size_t i = 0; i < buf_size; i++) {
        SoapySDR_logf(SOAPY_SDR_INFO, "sending command: %02x", buf[i]);
    }
    if ((res = FT_WritePipe(handle, CMD_OUT, buf, buf_size, &count, 0)) != FT_OK) {
                printf("FT_WritePipe: %d\n", res);
                return false;
        }
        if (count != buf_size) {
                printf("FT_WritePipe wrote %d bytes, but we wanted %d\n", count, buf_size);
                return false;
        }
        return true;
}

std::vector<uint8_t>
IcomIQ::read_reply()
{
        uint8_t buf[DEFAULT_BUFFER_LENGTH];
        DWORD i;
        int res;
        DWORD count;

        if ((res = FT_ReadPipe(handle, CMD_IN, buf, sizeof(buf), &count, 0)) != FT_OK) {
                printf("FT_ReadPipe: %d\n", res);
                res = FT_AbortPipe(handle, CMD_IN);
                printf("FT_AbortPipe: %d\n", res);
                return {};
        }

/*
        switch (buf[CMD_INDEX]) {
                case 0x1a: switch (buf[SUBCMD_INDEX]) {
                                   case 0x0a: printf("OVF: %d", buf[DATA_INDEX]); break;
                                   case 0x0b: printf("IQ data output: %d", buf[DATA_INDEX]); break;
                           }
                           break;
                case 0x1c: switch (buf[SUBCMD_INDEX]) {
                                   case 0x00: printf("TX: %d", buf[DATA_INDEX]); break;
                                   case 0x02: printf("XFC: %d", buf[DATA_INDEX]); break;
                           }
                           break;
                case 0xfa: printf("NG"); break;
                case 0xfb: printf("OK"); break;
        }
        printf("\n");
        */

        std::vector<uint8_t> vect = {};
        for (i = 0; i < count; i++)
        {
            vect.push_back(buf[i]);
        }

        return vect;
}

int
IcomIQ::icom_cmd(std::vector <uint8_t> cmd, std::vector<uint8_t> &reply)
{
    bool res = send_cmd(cmd);
    reply = read_reply();
    for (size_t i = 0; i < reply.size(); i++) {
        SoapySDR_logf(SOAPY_SDR_INFO, "reply %02x", reply[i]);
    }
    return reply[CMD_INDEX];
}


/*******************************************************************
 * Identification API
 ******************************************************************/

std::string IcomIQ::getDriverKey(void) const
{
    return "IcomIQ";
}

std::string IcomIQ::getHardwareKey(void) const
{
    return "IC-7610";
}

SoapySDR::Kwargs IcomIQ::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["origin"] = "https://github.com/df7cb/SoapyIcomIQ";
    args["serial"] = serial;

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t IcomIQ::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

bool IcomIQ::getFullDuplex(const int direction, const size_t channel) const
{
    return false;
}

/*******************************************************************
 * Stream API
 ******************************************************************/

std::vector<std::string> IcomIQ::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    return formats;
}

std::string IcomIQ::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    //check that direction is SOAPY_SDR_RX
     if (direction != SOAPY_SDR_RX) {
         throw std::runtime_error("RTL-SDR is RX only, use SOAPY_SDR_RX");
     }

     fullScale = 1<<15;
     return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList IcomIQ::getStreamArgsInfo(const int direction, const size_t channel) const {
    //check that direction is SOAPY_SDR_RX
     if (direction != SOAPY_SDR_RX) {
         throw std::runtime_error("RTL-SDR is RX only, use SOAPY_SDR_RX");
     }

    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
    bufflenArg.name = "Buffer Size";
    bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
    bufflenArg.units = "bytes";
    bufflenArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(bufflenArg);

    SoapySDR::ArgInfo buffersArg;
    buffersArg.key = "buffers";
    buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
    buffersArg.name = "Ring buffers";
    buffersArg.description = "Number of buffers in the ring.";
    buffersArg.units = "buffers";
    buffersArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(buffersArg);

    SoapySDR::ArgInfo asyncbuffsArg;
    asyncbuffsArg.key = "asyncBuffs";
    asyncbuffsArg.value = "0";
    asyncbuffsArg.name = "Async buffers";
    asyncbuffsArg.description = "Number of async usb buffers (advanced).";
    asyncbuffsArg.units = "buffers";
    asyncbuffsArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(asyncbuffsArg);

    return streamArgs;
}

SoapySDR::Stream *IcomIQ::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("RTL-SDR is RX only, use SOAPY_SDR_RX");
    }

    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format
    if (format == SOAPY_SDR_CS16)
    {
        rxFormat = format;
    }
    else if (format == SOAPY_SDR_CF32)
    {
        rxFormat = format;
    }
    else
    {
        throw std::runtime_error(
                "setupStream invalid format '" + format
                        + "' -- Only CS16 is supported by SoapyIcomIQ module.");
    }

    bufferLength = DEFAULT_BUFFER_LENGTH;
    if (args.count("bufflen") != 0)
    {
        try
        {
            int bufferLength_in = std::stoi(args.at("bufflen"));
            if (bufferLength_in > 0)
            {
                bufferLength = bufferLength_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using buffer length %d", bufferLength);

    numBuffers = DEFAULT_NUM_BUFFERS;
    if (args.count("buffers") != 0)
    {
        try
        {
            int numBuffers_in = std::stoi(args.at("buffers"));
            if (numBuffers_in > 0)
            {
                numBuffers = numBuffers_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using %d buffers", numBuffers);

    asyncBuffs = 0;
    if (args.count("asyncBuffs") != 0)
    {
        try
        {
            int asyncBuffs_in = std::stoi(args.at("asyncBuffs"));
            if (asyncBuffs_in > 0)
            {
                asyncBuffs = asyncBuffs_in;
            }
        }
        catch (const std::invalid_argument &){}
    }

/*
    //clear async fifo counts
    _buf_tail = 0;
    _buf_count = 0;
    _buf_head = 0;

    //allocate buffers
    _buffs.resize(numBuffers);
    for (auto &buff : _buffs) buff.data.reserve(bufferLength);
    for (auto &buff : _buffs) buff.data.resize(bufferLength);
    */

    return (SoapySDR::Stream *) this;
}

void IcomIQ::closeStream(SoapySDR::Stream *stream)
{
    this->deactivateStream(stream, 0, 0);
    //_buffs.clear();
}

size_t IcomIQ::getStreamMTU(SoapySDR::Stream *stream) const
{
    return bufferLength / BYTES_PER_SAMPLE;
}

int IcomIQ::activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems)
{
    /*
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    resetBuffer = true;
    bufferedElems = 0;

    //start the async thread
    if (not _rx_async_thread.joinable())
    {
        rtlsdr_reset_buffer(dev);
        _rx_async_thread = std::thread(&IcomIQ::rx_async_operation, this);
    }
    */

    std::vector<uint8_t> reply;
    icom_cmd({0x1a, 0x0b, 0x01}, reply);

    return 0;
}

int IcomIQ::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    /*
    if (_rx_async_thread.joinable())
    {
        rtlsdr_cancel_async(dev);
        _rx_async_thread.join();
    }
    */
    std::vector<uint8_t> reply;
    icom_cmd({0x1a, 0x0b, 0x00}, reply);
    return 0;
}

int IcomIQ::readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs)
{
    //drop remainder buffer on reset
    if (resetBuffer and bufferedElems != 0)
    {
        bufferedElems = 0;
        this->releaseReadBuffer(stream, _currentHandle);
    }

    uint8_t buf[DEFAULT_BUFFER_LENGTH];
        int res;
        DWORD count;

        if ((res = FT_ReadPipe(handle, IQ_IN, buf, sizeof(buf), &count, 0)) != FT_OK) {
                        printf("FT_ReadPipe: %d\n", res);
                        return 0;
                }

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

    if (rxFormat == SOAPY_SDR_CS16)
    {
        int16_t *fsource = (int16_t *) buf;
        int16_t *ftarget = (int16_t *) buff0;

        for (size_t i = 0; i < count / 2; i++) {
            ftarget[i] = fsource[i];
        }
    }
    else if (rxFormat == SOAPY_SDR_CF32)
    {
        int16_t *fsource = (int16_t *) buf;
        float *ftarget = (float *) buff0;

        for (size_t i = 0; i < count / 2; i++) {
            ftarget[i] = fsource[i] / 32768.0;
        }
    }

    return count / BYTES_PER_SAMPLE;


    /*
    //are elements left in the buffer? if not, do a new read.
    if (bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
        if (ret < 0) return ret;
        bufferedElems = ret;
    }

    //otherwise just update return time to the current tick count
    else
    {
        flags |= SOAPY_SDR_HAS_TIME;
        timeNs = SoapySDR::ticksToTimeNs(bufTicks, sampleRate);
    }

    size_t returnedElems = std::min(bufferedElems, numElems);

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

    //are elements left in the buffer? if not, do a new read.
    if (bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
        if (ret < 0) return ret;
        bufferedElems = ret;
    }

    //otherwise just update return time to the current tick count
    else
    {
        flags |= SOAPY_SDR_HAS_TIME;
        timeNs = SoapySDR::ticksToTimeNs(bufTicks, sampleRate);
    }

*/
    size_t returnedElems = std::min(bufferedElems, numElems);

    /* TODO */

/*
    //bump variables for next call into readStream
    bufferedElems -= returnedElems;
    _currentBuff += returnedElems*BYTES_PER_SAMPLE;
    bufTicks += returnedElems; //for the next call to readStream if there is a remainder
    */

    //return number of elements written to buff0
    if (bufferedElems != 0) flags |= SOAPY_SDR_MORE_FRAGMENTS;
    else this->releaseReadBuffer(stream, _currentHandle);
    return returnedElems;
}


/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> IcomIQ::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("ANT1");
    antennas.push_back("ANT2");
    antennas.push_back("RX");
    return antennas;
}

void IcomIQ::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("setAntena failed: TODO"); // TODO
    }
}

std::string IcomIQ::getAntenna(const int direction, const size_t channel) const
{
    return "RX"; // TODO
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

static inline int
bcd_digit(uint32_t n, int value)
{
    return (n / value) % 10;
}

static inline int
bcd_digits(uint32_t n, int value)
{
    return 0x10 * bcd_digit(n, 10 * value) | bcd_digit(n, value);
}

void IcomIQ::setFrequency(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double frequency,
        const SoapySDR::Kwargs &args)
{
    if (name == "RF")
    {
        uint32_t f = frequency;
        SoapySDR_logf(SOAPY_SDR_INFO, "Setting center freq: %d", f);

        std::vector<uint8_t> cmd = {0x25, 0x00};
        cmd.push_back(bcd_digits(f, 1));
        cmd.push_back(bcd_digits(f, 100));
        cmd.push_back(bcd_digits(f, 10000));
        cmd.push_back(bcd_digits(f, 1000000));
        cmd.push_back(bcd_digits(f, 100000000));

        std::vector<uint8_t> reply;
        icom_cmd(cmd, reply);
    }
    else if (name == "CORR")
    {
        /*
        int r = rtlsdr_set_freq_correction(dev, (int)frequency);
        if (r == -2)
        {
            return; // CORR didn't actually change, we are done
        }
        if (r != 0)
        {
            throw std::runtime_error("setFrequencyCorrection failed");
        }
        ppm = rtlsdr_get_freq_correction(dev);
        */
    }
    else
        SoapySDR_logf(SOAPY_SDR_INFO, "getFrequency: unknown name %s", name.c_str());
}

static inline int
read_bcd(int n)
{
    return 10 * ((n & 0xf0) >> 4) + (n & 0x0f);
}

double IcomIQ::getFrequency(const int direction, const size_t channel, const std::string &name)
{
    if (name == "RF")
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Getting center freq");
        std::vector<uint8_t> reply;
        icom_cmd({0x25, 0x00}, reply);

        uint32_t f = read_bcd(reply[DATA_INDEX])   * 1 +
                     read_bcd(reply[DATA_INDEX+1]) * 100 +
                     read_bcd(reply[DATA_INDEX+2]) * 10000 +
                     read_bcd(reply[DATA_INDEX+3]) * 1000000 +
                     read_bcd(reply[DATA_INDEX+4]) * 100000000;

        return (double) f;
    }
    else if (name == "CORR")
    {
        return (double) 1; // TODO ppm;
    }
    else
        SoapySDR_logf(SOAPY_SDR_INFO, "getFrequency: unknown name %s", name.c_str());

    return 0;
}

std::vector<std::string> IcomIQ::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    //names.push_back("TX");
    //names.push_back("CORR");
    return names;
}

SoapySDR::RangeList IcomIQ::getFrequencyRange(
        const int direction,
        const size_t channel,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        //results.push_back(SoapySDR::Range(30000, 60000000));
        results.push_back(SoapySDR::Range(0, 60000000));
    }
    if (name == "TX")
    {
        results.push_back(SoapySDR::Range(135700, 137800));
        results.push_back(SoapySDR::Range(1800000, 29700000)); // TODO: fill in bands
        results.push_back(SoapySDR::Range(50000000, 54000000));
    }
    if (name == "CORR")
    {
        // TODO results.push_back(SoapySDR::Range(-1000, 1000));
    }
    return results;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void IcomIQ::setSampleRate(const int direction, const size_t channel, const double rate)
{
    return;
}

double IcomIQ::getSampleRate(const int direction, const size_t channel) const
{
    return 1920000;
}

std::vector<double> IcomIQ::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;
    results.push_back(1920000);
    return results;
}


/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findIcomIQ(const SoapySDR::Kwargs &args)
{
    //(void)args;
    FT_DEVICE_LIST_INFO_NODE nodes[16];
    DWORD count;
    int res;
    std::vector<SoapySDR::Kwargs> results;

    if ((res = FT_CreateDeviceInfoList(&count)) != FT_OK) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "FT_CreateDeviceInfoList failed: %d\n", res);
        return SoapySDR::KwargsList();
    }

    if ((res = FT_GetDeviceInfoList(nodes, &count)) != FT_OK) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "FT_GetDeviceInfoList failed: %d\n", res);
        return SoapySDR::KwargsList();
    }

    if (count == 0) {
        SoapySDR_logf(SOAPY_SDR_INFO, "No FTDI devices found");
    }

    for (DWORD i = 0; i < count; i++) {
        SoapySDR_logf(SOAPY_SDR_INFO, "Device[%d]", i);
        SoapySDR_logf(SOAPY_SDR_INFO, "\tFlags: 0x%x %s | Type: %d | ID: 0x%08X",
                nodes[i].Flags,
                nodes[i].Flags & FT_FLAGS_SUPERSPEED ? "[USB 3]" :
                nodes[i].Flags & FT_FLAGS_HISPEED ? "[USB 2]" :
                nodes[i].Flags & FT_FLAGS_OPENED ? "[OPENED]" : "",
                nodes[i].Type,
                nodes[i].ID);
        SoapySDR_logf(SOAPY_SDR_INFO, "SerialNumber=%s\n", nodes[i].SerialNumber);
        SoapySDR_logf(SOAPY_SDR_INFO, "Description=%s\n", nodes[i].Description);

        SoapySDR::Kwargs devInfo;
        devInfo["label"] = std::string(nodes[i].Description) + " :: " + nodes[i].SerialNumber;
        devInfo["product"] = nodes[i].Description;
        devInfo["serial"] = nodes[i].SerialNumber;
        devInfo["manufacturer"] = "Icom";
        //devInfo["tuner"] = get_tuner(serial, i);

        //filtering by serial
        if (args.count("serial") != 0 and args.at("serial") != nodes[i].SerialNumber) continue;

        results.push_back(devInfo);
    }

    return results;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeIcomIQ(const SoapySDR::Kwargs &args)
{
    (void)args;
    //create an instance of the device object given the args
    //here we will translate args into something used in the constructor
    return new IcomIQ(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerIcomIQ("icomiq", &findIcomIQ, &makeIcomIQ, SOAPY_SDR_ABI_VERSION);
